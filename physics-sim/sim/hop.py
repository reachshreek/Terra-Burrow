"""Run one hop and measure what happened.

A hop has four phases:
  1. wind    - get each rear leg ready (wind-and-latch legs reel themselves into
               their ratchet tooth; direct-drive servo legs are ready at once)
  2. settle  - hold still until the bunny stops moving
  3. push    - fire each rear leg. Left and right can push by different amounts
               and fire at different times, which is how the bunny turns.
  4. land    - legs fold back; wait until everything stops moving

Run:  python sim/hop.py                                 (one straight hop, BOM design)
      python sim/hop.py --design latch 1.0 1.0 0.02     (left push, right push, right-leg delay in s)
"""

import argparse
import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from config import DESIGNS, model_path
from front_legs import FrontLegs
from legs import make_leg, winding_report
from servo import servo_from_design

MAX_WIND_TIME = 3.0  # s, give up winding after this
SETTLE_TIME = 0.6  # s, still time before the push
LAND_TIME = 1.2  # s, minimum time after the push before measuring
MAX_HOP_TIME = 3.0  # s, give up waiting for it to settle
UPRIGHT_LIMIT_DEG = 30.0  # final tilt beyond this counts as a fall
SIDES = ("left", "right")


@dataclass
class HopCommand:
    push_left: float = 1.0  # 0 = no push, 1 = full push (latch drive: fraction of full spring energy)
    push_right: float = 1.0
    right_delay: float = 0.0  # s; positive = right leg fires later, negative = earlier


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
    """The MuJoCo model plus the two rear-leg drivers (and the front legs, if driven)."""

    def __init__(self, design_name="bom", model=None, servo_left=None, servo_right=None, sensor_seed=0):
        design = DESIGNS[design_name]
        self.design = design
        self.model = model if model is not None else mujoco.MjModel.from_xml_path(str(model_path(design_name)))
        self.data = mujoco.MjData(self.model)
        self.servos = {
            "left": servo_left or servo_from_design(design),
            "right": servo_right or servo_from_design(design),
        }
        self.legs = {side: make_leg(design, self.servos[side]) for side in SIDES}
        self.sensor_seed = sensor_seed
        self.front = self.make_front_legs()

        def name_id(kind, name):
            return mujoco.mj_name2id(self.model, kind, name)

        def geom_id(name):
            return name_id(mujoco.mjtObj.mjOBJ_GEOM, name)

        def joint_id(name):
            return name_id(mujoco.mjtObj.mjOBJ_JOINT, name)

        def actuator_id(name):
            return name_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)

        self.floor = geom_id("floor")
        self.torso = name_id(mujoco.mjtObj.mjOBJ_BODY, "torso")
        self.body_geoms = {geom_id("torso_shell"), geom_id("head")}
        self.feet = {
            "rear_left": geom_id("rear_foot_left"),
            "rear_right": geom_id("rear_foot_right"),
            "front_left": geom_id("front_foot_left"),
            "front_right": geom_id("front_foot_right"),
        }
        self.hip_qpos = {s: self.model.jnt_qposadr[joint_id(f"hip_{s}")] for s in SIDES}
        self.hip_qvel = {s: self.model.jnt_dofadr[joint_id(f"hip_{s}")] for s in SIDES}
        self.servo_actuator = {s: actuator_id(f"servo_{s}") for s in SIDES}

        self.has_latch = design.drive == "latch"
        if self.has_latch:
            self.spring_actuator = {s: actuator_id(f"spring_{s}") for s in SIDES}
            self.latch_constraint = {
                s: name_id(mujoco.mjtObj.mjOBJ_EQUALITY, f"latch_{s}") for s in SIDES
            }

    # --- state helpers ---------------------------------------------------

    def reset_to_crouch(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.legs = {side: make_leg(self.design, self.servos[side]) for side in SIDES}
        self.front = self.make_front_legs()

    def make_front_legs(self):
        if self.design.front_drive != "servo":
            return None
        return FrontLegs(self.design, self.model, self.data, rng=np.random.default_rng(self.sensor_seed))

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
        for side, leg in self.legs.items():
            angle = self.data.qpos[self.hip_qpos[side]]
            speed = self.data.qvel[self.hip_qvel[side]]
            servo_torque, spring_torque = leg.update(angle, speed, dt)
            self.data.ctrl[self.servo_actuator[side]] = servo_torque

            if self.has_latch:
                self.data.ctrl[self.spring_actuator[side]] = spring_torque
                # The ratchet pawl: lock the hip at the angle where it caught
                constraint = self.latch_constraint[side]
                self.model.eq_data[constraint][0] = leg.latch_angle
                self.data.eq_active[constraint] = leg.latched
        if self.front:
            self.front.step(dt)
        mujoco.mj_step(self.model, self.data)

    def run(self, duration, on_step=None):
        steps = int(duration / self.model.opt.timestep)
        for _ in range(steps):
            self.step()
            if on_step:
                on_step(self)

    def is_still(self):
        return np.linalg.norm(self.data.qvel[:6]) < 0.02


def simulate_hop(bunny: Bunny, command: HopCommand, on_step=None) -> HopResult:
    """Do one hop starting from the bunny's current state and measure it."""
    result = HopResult(command)
    dt = bunny.model.opt.timestep
    legs = bunny.legs

    # 1. Wind: get each leg ready for its push
    legs["left"].prepare(command.push_left)
    legs["right"].prepare(command.push_right)
    while not all(leg.is_ready() for leg in legs.values()) and result.wind_time < MAX_WIND_TIME:
        bunny.step()
        if on_step:
            on_step(bunny)
        result.wind_time += dt

    # 2. Settle
    bunny.run(SETTLE_TIME, on_step)
    start_position = bunny.torso_position()
    start_heading = bunny.heading()

    # 3-4. Push and land
    fire_time = {"left": max(0.0, -command.right_delay), "right": max(0.0, command.right_delay)}
    fired = {"left": False, "right": False}
    if bunny.front:
        bunny.front.fire()

    t = 0.0
    while t < MAX_HOP_TIME:
        for side in SIDES:
            if not fired[side] and t >= fire_time[side]:
                legs[side].fire()
                fired[side] = True

        # Measurements
        touching = bunny.floor_contacts()
        if touching & bunny.body_geoms:
            result.body_hit_floor = True
        if not touching & set(bunny.feet.values()):
            result.flight_time += dt
        if not touching & {bunny.feet["front_left"], bunny.feet["front_right"]}:
            result.front_lift_time += dt
        rise = bunny.torso_position()[2] - start_position[2]
        result.apex_height = max(result.apex_height, rise)
        result.max_tilt_deg = max(result.max_tilt_deg, bunny.tilt_deg())
        result.peak_current = max(result.peak_current, *(leg.current for leg in legs.values()))
        if bunny.front:
            result.peak_current = max(result.peak_current, bunny.front.current)

        bunny.step()
        if on_step:
            on_step(bunny)
        t += dt

        all_tucked = all(fired.values()) and all(leg.is_tucked() for leg in legs.values())
        if all_tucked and t > LAND_TIME and bunny.is_still():
            break

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
    parser.add_argument("--design", default="bom", choices=DESIGNS)
    parser.add_argument("push_left", nargs="?", type=float, default=1.0)
    parser.add_argument("push_right", nargs="?", type=float, default=1.0)
    parser.add_argument("right_delay", nargs="?", type=float, default=0.0)
    args = parser.parse_args()

    bunny = Bunny(args.design)
    bunny.reset_to_crouch()
    print_report(simulate_hop(bunny, HopCommand(args.push_left, args.push_right, args.right_delay)))
    if bunny.has_latch:
        print_winding(bunny.design)
