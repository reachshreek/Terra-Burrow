"""The hop machinery: everything the controller does that you shouldn't need to touch.

BaselineController runs the attitude estimator, every leg's phase logic, and the
hop and bounding sequences, and fills in a HopRecord for every hop. The few
decisions worth changing go through methods that bunny_controller.py
overrides:

  plan_hop(command)          -> HopPlan    what this launch does
  bound_hop()                -> command    the next hop of a bound (planned at each landing)
  retime_hop(plan)           -> HopPlan    last look at a plan, at the moment of launch
  ready_to_launch(inputs, bounding) -> bool   when to fire
  front_leg_goals(goals, inputs) -> goals  front-leg servo goals, every tick
  front_swing_goals(inputs)  -> goals      front shoulder swing servos, every tick (front_swing designs)
  on_hop_finished(record)                  what the robot measured, after each hop

Their defaults here reproduce the controller as it was: do exactly what the
command says, fire once still (or, bounding, once both rear feet are down),
and run the front legs' fixed push/tuck/reach/level sequence.
"""

import math

from .estimator import STILL_TIME, Estimator
from .interface import (DT, SIDES, BoundCommand, HopCommand, HopPlan, HopRecord, Inputs, Outputs,
                        ServoCommand)
from .legs import FREE_FALL_BELOW_G, FrontLegLogic, LatchLegLogic

MAX_WIND_TIME = 3.0  # s, fire anyway after this
SETTLE_STILL = 0.4  # s of IMU stillness before a hop from standing
SETTLE_MAX = 2.0  # s, hop anyway after this
LAND_MIN = 0.5  # s after firing before a hop can count as finished
# Touchdown, for the HopRecord: reeling the rear legs back in mid-flight jolts the body
# 1.8-2.4 g, real landings 5 g and up. (The front legs' own TOUCHDOWN_ABOVE_G is lower.)
RECORD_TOUCHDOWN_G = 3.5
HOP_MAX = 3.0  # s after firing, give up waiting to be still

# Bounding: relaunch out of each landing
BOUND_MIN_GROUND = 0.03  # s with both rear feet down before relaunching
BOUND_WAIT_MAX = 1.0  # s, launch anyway after waiting this long


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class BaselineController:
    def __init__(self, design, front_stance_angle=None):
        self.design = design
        self.estimator = Estimator()
        self.legs = {s: LatchLegLogic(design, f"rear_{s}") for s in SIDES}
        self.front = FrontLegLogic(design, front_stance_angle)
        self.status = "idle"
        self.time_in_status = 0.0
        self.hops_started = 0
        self.plan = None  # the HopPlan being carried out
        self.bound = None  # the BoundCommand being carried out, if any
        self.bounds_left = 0
        self.fire_at = {}
        self.fired = {}
        self.record = None  # HopRecord of the hop in progress
        self.history = []  # finished HopRecords, newest last
        self.hop_time = 0.0  # s since the current hop launched
        self.launch_yaw = 0.0
        self.release_delay = {s: 0.0 for s in SIDES}
        self.waiting_command = None  # a command that arrived before the attitude estimate was ready
        self.bound_hops_done = 0  # hops launched in this bound (a mid-bound update doesn't reset it)

    # --- hooks (bunny_controller.py overrides these) -----------------------

    def plan_hop(self, command: HopCommand) -> HopPlan:
        return HopPlan(command.push_left, command.push_right, command.right_delay)

    def bound_hop(self):
        return HopCommand(self.bound.push, self.bound.push, 0.0)

    def retime_hop(self, plan: HopPlan) -> HopPlan:
        return plan

    def ready_to_launch(self, inputs: Inputs, bounding: bool) -> bool:
        est = self.estimator
        if not bounding:
            return est.still_time >= SETTLE_STILL or self.time_in_status > SETTLE_MAX
        feet_down = all(inputs.foot_switch.values())
        return (feet_down and self.time_in_status >= BOUND_MIN_GROUND) or self.time_in_status > BOUND_WAIT_MAX

    def front_leg_goals(self, goals: dict, inputs: Inputs) -> dict:
        return goals

    def front_swing_goals(self, inputs: Inputs) -> dict:
        return {s: ServoCommand(0.0) for s in SIDES}

    def on_hop_finished(self, record: HopRecord):
        pass

    # --- sequencing ---------------------------------------------------------

    def set_status(self, status):
        self.status = status
        self.time_in_status = 0.0

    def start_hop(self, command: HopCommand = None):
        """command None: the next hop of the bound."""
        self.plan = self.plan_hop(self.bound_hop() if command is None else command)
        self.legs["left"].prepare(self.plan.push_left)
        self.legs["right"].prepare(self.plan.push_right)
        self.set_status("winding")

    def launch(self):
        self.plan = self.retime_hop(self.plan)
        self.release_delay = {s: 0.0 for s in SIDES}
        if self.bound:
            self.bound_hops_done += 1
        d = self.plan.right_delay
        # A negative front_delay means the front legs push first: hold both rear releases back instead
        rear_hold = max(0.0, -self.plan.front_delay)
        self.fire_at = {"left": rear_hold + max(0.0, -d), "right": rear_hold + max(0.0, d)}
        self.fired = {s: False for s in SIDES}
        self.front.fire(max(0.0, self.plan.front_delay), self.plan.front_push)
        self.hops_started += 1
        roll, pitch = self.estimator.roll_pitch()
        self.record = HopRecord(self.plan, launch_pitch=pitch, launch_roll=roll)
        self.launch_yaw = self.estimator.yaw()
        self.estimator.reset_velocity()
        self.hop_time = 0.0
        self.set_status("hopping")

    def track_hop(self, inputs):
        """Fill in liftoff and touchdown in the current HopRecord from the IMU and foot switches."""
        r, est = self.record, self.estimator
        if r is None:
            return
        self.hop_time += DT
        g = est.g_felt()
        roll, pitch = est.roll_pitch()
        if math.isnan(r.liftoff_time):
            if g < FREE_FALL_BELOW_G:
                r.liftoff_time, r.liftoff_pitch = self.hop_time, pitch
                r.liftoff_pitch_rate, r.liftoff_roll_rate = est.gyro[1], est.gyro[0]
                r.liftoff_speed, r.liftoff_climb = est.forward_speed(self.launch_yaw), est.velocity[2]
        elif math.isnan(r.touchdown_time):
            if g > RECORD_TOUCHDOWN_G or any(inputs.foot_switch.values()):
                r.touchdown_time, r.touchdown_pitch, r.touchdown_roll = self.hop_time, pitch, roll
                r.touchdown_pitch_rate = est.gyro[1]
        if not math.isnan(r.liftoff_time):
            r.peak_g = max(r.peak_g, g)

    def finish_hop(self):
        r = self.record
        if r is None:
            return
        r.settled_roll, r.settled_pitch = self.estimator.roll_pitch()
        r.yaw_change = wrap(self.estimator.yaw() - self.launch_yaw)
        self.history.append(r)
        self.record = None
        self.on_hop_finished(r)

    # --- the tick -----------------------------------------------------------

    def update(self, inputs: Inputs) -> Outputs:
        est = self.estimator
        est.update(inputs.gyro, inputs.accel)
        self.time_in_status += DT
        pos = inputs.servo_position

        # Commands wait until the IMU has given a first attitude estimate (just after power-up)
        command = inputs.command or self.waiting_command
        self.waiting_command = None
        if command is not None and not est.ready():
            self.waiting_command, command = command, None
        if isinstance(command, HopCommand):
            self.bound = None
            self.start_hop(command)
        elif isinstance(command, BoundCommand) and self.bound is not None and self.status != "idle":
            self.bound, self.bounds_left = command, command.hops  # already bounding: carry on with the new orders
        elif isinstance(command, BoundCommand):
            self.bound = command
            self.bounds_left = command.hops
            self.bound_hops_done = 0
            self.start_hop()

        if self.status == "winding":
            ready = all(leg.ready(pos[f"rear_{s}"]) for s, leg in self.legs.items())
            if ready or self.time_in_status > MAX_WIND_TIME:
                self.set_status("settling")
        if self.status == "settling" and self.ready_to_launch(inputs, bounding=self.bound is not None):
            if self.bound:
                self.bounds_left -= 1
            self.launch()
        if self.status == "hopping":
            self.track_hop(inputs)
            for s in SIDES:
                # Fire in the tick whose window holds the release time, offset to the exact moment
                if not self.fired[s] and self.fire_at[s] < self.time_in_status + DT - 1e-9:
                    self.legs[s].fire()
                    self.fired[s] = True
                    self.release_delay[s] = max(0.0, self.fire_at[s] - self.time_in_status)
            legs_done = all(self.fired.values()) and all(leg.done() for leg in self.legs.values())
            if self.bound and self.bounds_left > 0 and self.time_in_status > HOP_MAX:
                self.finish_hop()  # never got both rear feet down (on its side?): stop bounding
                self.bound = None
                self.set_status("idle")
            elif self.bound and self.bounds_left > 0:
                if legs_done and all(inputs.foot_switch.values()) and self.time_in_status > 0.1:
                    self.finish_hop()
                    self.start_hop()
            elif (legs_done and self.time_in_status > LAND_MIN and est.still_time >= STILL_TIME) \
                    or self.time_in_status > HOP_MAX:
                self.finish_hop()
                self.bound = None
                self.set_status("idle")

        servo, release = {}, {}
        for s, leg in self.legs.items():
            servo[f"rear_{s}"], release[s] = leg.tick(pos[f"rear_{s}"])
        goals = self.front.tick(est, {s: pos.get(f"front_{s}") for s in SIDES})
        for s, command in self.front_leg_goals(goals, inputs).items():
            servo[f"front_{s}"] = command
        if self.design.front_swing:
            for s, command in self.front_swing_goals(inputs).items():
                servo[f"front_swing_{s}"] = command

        roll, pitch = est.roll_pitch()
        status = "starting" if self.waiting_command is not None else self.status
        return Outputs(servo=servo, release=release, release_delay=dict(self.release_delay),
                       status=status, hops_started=self.hops_started,
                       roll=roll, pitch=pitch, yaw=est.yaw())
