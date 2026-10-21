"""Run one hop and measure what happened.

Bunny is the simulated robot: the MuJoCo model, its hardware (hardware.py) and
its controller (bunny_controller.py, or with controller_class=BaselineController
the controller without its control laws, for a before/after). Every controller tick, the hardware's
sensor readings go into the controller and its servo commands go back out.
simulate_hop() only sends the controller a HopCommand and watches; the
controller does the wind -> settle -> fire -> land sequence itself.

The measurements here (distance, tilt, ...) use the simulator's true state.
They score the hop, like a motion-capture rig would; the robot never sees them.

Run:  python sim/hop.py                     (one straight full hop)
      python sim/hop.py 1.0 0.7 0.02        (left push, right push, right-leg delay in s)
      python sim/hop.py --no-controller     (the same, with BaselineController: no control laws)
"""

import argparse
import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from build_model import front_stance_angle
from config import DEFAULT_DESIGN, DESIGNS, model_path
from hardware import RobotHardware
from legs import winding_report
from bunny_controller import BunnyController
from controller import DT as CONTROL_DT
from controller import BaselineController, BoundCommand, HopCommand  # noqa: F401  (re-exported)
from servo import servo_from_design

MAX_PREP_TIME = 6.0  # s, give up if the controller never starts the hop
MAX_HOP_TIME = 4.0  # s after the push starts, give up waiting for it to finish
UPRIGHT_LIMIT_DEG = 30.0  # final tilt beyond this counts as a fall
SIDES = ("left", "right")


@dataclass
class HopResult:
    command: HopCommand
    distance: float = 0.0  # m, how far the torso moved
    direction_deg: float = 0.0  # travel direction relative to the starting heading (+ = left)
    heading_change_deg: float = 0.0  # how much the body turned (+ = left)
    apex_height: float = 0.0  # m, highest rise of the torso above its start height
    flight_time: float = 0.0  # s, total time with all four feet off the ground
    front_lift_time: float = 0.0  # s, total time with both front feet off the ground
    wind_time: float = 0.0  # s, time spent getting the legs ready before this hop
    final_tilt_deg: float = 0.0
    max_tilt_deg: float = 0.0
    body_hit_floor: bool = False
    peak_current: float = 0.0  # A, highest single-servo current
    upright: bool = field(init=False, default=False)

    def finish(self):
        self.upright = self.final_tilt_deg < UPRIGHT_LIMIT_DEG and not self.body_hit_floor
        return self


class Bunny:
    """The simulated robot: MuJoCo model + hardware + controller."""

    def __init__(self, design_name=DEFAULT_DESIGN, model=None, servo_specs=None, sensor_seed=0,
                 controller_class=None):
        """servo_specs: optional {servo name: Servo} to vary individual servos (see sweep.py).
        sensor_seed: everything random about this particular robot (IMU errors,
        release-servo speeds, PWM phase, bus latencies).
        controller_class: None = BunnyController; BaselineController turns the control laws off."""
        design = DESIGNS[design_name]
        self.design = design
        self.model = model if model is not None else mujoco.MjModel.from_xml_path(str(model_path(design_name)))
        self.data = mujoco.MjData(self.model)
        self.front_stance = front_stance_angle(design)
        self.controller_class = controller_class
        names = [f"rear_{s}" for s in SIDES] + [f"front_{s}" for s in SIDES]
        self.servo_specs = {n: servo_from_design(design) for n in names}
        self.servo_specs.update(servo_specs or {})
        self.sensor_seed = sensor_seed

        def name_id(kind, name):
            return mujoco.mj_name2id(self.model, kind, name)

        def geom_id(name):
            return name_id(mujoco.mjtObj.mjOBJ_GEOM, name)

        self.floor = geom_id("floor")
        self.torso = name_id(mujoco.mjtObj.mjOBJ_BODY, "torso")
        self.body_geoms = {geom_id("torso_shell"), geom_id("head")}
        self.feet = {
            "rear_left": geom_id("rear_foot_left"),
            "rear_right": geom_id("rear_foot_right"),
            "front_left": geom_id("front_foot_left"),
            "front_right": geom_id("front_foot_right"),
        }
        self.power_up()

    def power_up(self):
        """Fresh hardware state and a freshly booted controller."""
        rng = np.random.default_rng(self.sensor_seed)
        self.hardware = RobotHardware(self.design, self.model, self.data, rng, self.servo_specs, self.front_stance)
        self.controller = (self.controller_class or BunnyController)(self.design, self.front_stance)
        self.tick_timer = 0.0
        self.pending_command = None
        self.outputs = None

    def reset_to_crouch(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.power_up()

    # --- talking to the controller ---------------------------------------

    def send(self, command):
        """Queue a HopCommand or BoundCommand for the controller's next tick."""
        self.pending_command = command

    @property
    def status(self):
        return self.outputs.status if self.outputs else "idle"

    def estimated_yaw(self):
        return self.outputs.yaw if self.outputs else 0.0

    # --- true state, for scoring only -------------------------------------

    def torso_position(self):
        return self.data.xpos[self.torso].copy()

    def heading(self):
        """Yaw of the body's forward (x) axis, in radians."""
        rotation = self.data.xmat[self.torso].reshape(3, 3)
        return math.atan2(rotation[1, 0], rotation[0, 0])

    def tilt_deg(self):
        """Angle between the body's up axis and world up."""
        rotation = self.data.xmat[self.torso].reshape(3, 3)
        return math.degrees(math.acos(np.clip(rotation[2, 2], -1, 1)))

    def floor_contacts(self):
        """Set of geom ids currently touching the floor."""
        touching = set()
        for contact in self.data.contact[: self.data.ncon]:
            if contact.geom1 == self.floor:
                touching.add(contact.geom2)
            elif contact.geom2 == self.floor:
                touching.add(contact.geom1)
        return touching

    # --- stepping --------------------------------------------------------

    def step(self):
        dt = self.model.opt.timestep
        now = self.data.time
        self.current = self.hardware.step(dt, now)
        self.tick_timer -= dt
        if self.tick_timer <= 0:
            self.tick_timer += CONTROL_DT
            inputs = self.hardware.sample()
            inputs.command, self.pending_command = self.pending_command, None
            self.outputs = self.controller.update(inputs)
            self.hardware.apply(self.outputs, now)
        mujoco.mj_step(self.model, self.data)

    def run(self, duration, on_step=None):
        for _ in range(int(duration / self.model.opt.timestep)):
            self.step()
            if on_step:
                on_step(self)


def simulate_hop(bunny: Bunny, command: HopCommand, on_step=None) -> HopResult:
    """Send one hop command, wait for the controller to finish it, and measure it."""
    result = HopResult(command)
    dt = bunny.model.opt.timestep

    def step():
        bunny.step()
        if on_step:
            on_step(bunny)

    # Winding and settling, done by the controller
    bunny.send(command)
    while bunny.pending_command is not None:  # until the controller's next tick picks it up
        step()
    waited = 0.0
    while bunny.status in ("starting", "winding", "settling") and waited < MAX_PREP_TIME:
        if bunny.status == "winding":
            result.wind_time += dt
        step()
        waited += dt

    start_position = bunny.torso_position()
    start_heading = bunny.heading()

    # The push and the landing
    t = 0.0
    while bunny.status != "idle" and t < MAX_HOP_TIME:
        touching = bunny.floor_contacts()
        if touching & bunny.body_geoms:
            result.body_hit_floor = True
        if not touching & set(bunny.feet.values()):
            result.flight_time += dt
        if not touching & {bunny.feet["front_left"], bunny.feet["front_right"]}:
            result.front_lift_time += dt
        result.apex_height = max(result.apex_height, bunny.torso_position()[2] - start_position[2])
        result.max_tilt_deg = max(result.max_tilt_deg, bunny.tilt_deg())
        result.peak_current = max(result.peak_current, bunny.current)
        step()
        t += dt

    # Final measurements, in the frame of the starting heading
    moved = bunny.torso_position() - start_position
    cos_h, sin_h = math.cos(start_heading), math.sin(start_heading)
    forward = cos_h * moved[0] + sin_h * moved[1]
    sideways = -sin_h * moved[0] + cos_h * moved[1]

    result.distance = math.hypot(forward, sideways)
    result.direction_deg = math.degrees(math.atan2(sideways, forward))
    turned = bunny.heading() - start_heading
    result.heading_change_deg = math.degrees(math.atan2(math.sin(turned), math.cos(turned)))
    result.final_tilt_deg = bunny.tilt_deg()
    return result.finish()


def print_report(result: HopResult):
    c = result.command
    print(f"Command: left push {c.push_left:.2f}, right push {c.push_right:.2f}, "
          f"right delay {c.right_delay * 1000:+.0f} ms")
    print(f"  distance         {result.distance * 100:6.1f} cm")
    print(f"  direction        {result.direction_deg:+6.1f} deg (+ = left)")
    print(f"  heading change   {result.heading_change_deg:+6.1f} deg")
    print(f"  apex rise        {result.apex_height * 100:6.1f} cm")
    print(f"  flight time      {result.flight_time * 1000:6.0f} ms (all feet off the ground)")
    print(f"  front feet lift  {result.front_lift_time * 1000:6.0f} ms")
    print(f"  max tilt         {result.max_tilt_deg:6.1f} deg")
    print(f"  landed upright   {'yes' if result.upright else 'NO'}"
          f"{' (body hit the floor)' if result.body_hit_floor else ''}")
    print(f"  peak servo current {result.peak_current:4.1f} A per servo")


def print_winding(design):
    w = winding_report(design)
    print("Wind-and-latch, per leg at full push:")
    print(f"  spring energy      {w['energy_j']:5.2f} J")
    print(f"  peak hip torque    {w['peak_hip_torque_nm']:5.2f} N·m")
    print(f"  pawl force         {w['pawl_force_n']:5.0f} N at {design.latch_lever_radius * 1000:.0f} mm radius")
    print(f"  winder gearing     {w['winder_gear_ratio']:5.2f} : 1 (servo turns per hip turn)")
    print(f"  wind-up time       {w['wind_time_s']:5.2f} s from fully extended")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulate one hop and print what happened.")
    parser.add_argument("--design", default=DEFAULT_DESIGN, choices=DESIGNS)
    parser.add_argument("--no-controller", action="store_true", help="BaselineController: no control laws")
    parser.add_argument("push_left", nargs="?", type=float, default=1.0)
    parser.add_argument("push_right", nargs="?", type=float, default=1.0)
    parser.add_argument("right_delay", nargs="?", type=float, default=0.0)
    args = parser.parse_args()

    bunny = Bunny(args.design, controller_class=BaselineController if args.no_controller else None)
    bunny.reset_to_crouch()
    print_report(simulate_hop(bunny, HopCommand(args.push_left, args.push_right, args.right_delay)))
    print_winding(bunny.design)
