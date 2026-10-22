"""The bunny's controller: bigger, continuous, steerable hops.

Built on the machinery in controller/ (estimator, leg phases, sequencing). The control laws
(see README, "Controller", and controller_eval.py):

1. Front-lead feedforward. The front legs push FRONT_LEAD before the rear legs
   are released, lifting the nose first so the rear push doesn't throw the body
   into a spin it can't land from.

2. Steered bounding (BoundCommand.heading_deg). A heading loop, one correction
   per hop: turn = clip(heading - yaw, ±MAX_TURN_DEG). At each landing the teeth
   and release timing are picked from a model fitted on bounding hops
   (bound_map.py); once the teeth are wound, the release timing is picked again
   at the moment of launch, from the yaw measured then.

3. Raibert-style three-part control (designs with a front shoulder swing,
   config.front_swing; BoundCommand.height_cm / speed_cm_s turn loops a and b on):
   a. Height. Each hop's apex height is measured from its flight time
      (h = g T² / 8), and the next hop's rear push corrects toward the target:
          push += HEIGHT_GAIN * (height* - height)
   b. Speed. Each hop's forward speed is measured at liftoff (accelerometer,
      integrated over the push), and the front feet are placed for the next launch:
          swing_launch += SPEED_GAIN * (speed* - speed)
      (measured: front feet further forward at launch = faster, about 0.8 cm/s per degree;
      it only speeds up from there, within SPEED_SWING_RANGE_DEG: slow down with a lower height)
   c. Attitude. While the front feet are on the ground (push, landing, winding),
      the shoulder servos hold the body level, the hip-torque law of Raibert's hoppers:
          swing = swing_base + ATTITUDE_KP * pitch + ATTITUDE_KD * pitch_rate
      In flight the front feet are placed SWING_TOUCHDOWN_DEG forward for the landing,
      ahead of the body like Raibert's neutral point, so the landing doesn't pitch it nose-first.
"""

import dataclasses
from dataclasses import dataclass
import math

import numpy as np

from controller import BaselineController, HopCommand, HopPlan, HopRecord, Inputs, ServoCommand, SIDES, DT
from controller.bound_model import best_command, fit_model, possible_pushes

# --- 1. Front-lead feedforward ---
# Step test (16 robots, Pico timers): upright with a 0 / 15 / 30 ms lead was 100/100/100% at
# 7 N·m/rad, 100/100/94% at 10, and 31/94/75% at 13. Costs ~20% of distance at 13.
FRONT_LEAD = 0.05  # s

# --- 2. Steered bounding ---
MAX_TURN_DEG = 20.0  # most body turn asked of one bound hop
BOUND_LOWEST_PUSH = 0.5  # the bound map covers teeth from here up

# --- 3. Raibert-style control (front_swing designs) ---
HEIGHT_GAIN = 3.0  # push fraction per m of height error, per hop (~half of 1 / (16 cm per unit push))
SPEED_GAIN = 0.5  # rad of launch swing per m/s of speed error, per hop (~1/4 of 1 / (0.48 m/s per rad): no overshoot)
# Feet back of -5 deg at launch made spring-13 bounding fall over; to go slower, ask for a lower height instead
SPEED_SWING_RANGE_DEG = (-5.0, 20.0)
SWING_LAUNCH_DEG = 0.0  # where the front feet start, before the speed loop moves them
SWING_TOUCHDOWN_DEG = 28.0  # front-foot placement in flight, for the landing (+ = forward)
ATTITUDE_KP = 1.0  # rad of swing per rad of pitch (+ pitch = nose down: feet forward)
ATTITUDE_KD = 0.03  # rad of swing per rad/s of pitch rate
ATTITUDE_WHEN = "landing"  # "landing": from touchdown until the hop is done; "stance": also winding and the push
SWING_SLEW = 2.0  # rad/s, how fast the swing base moves while the feet are planted
SWING_READY = math.radians(3)  # launch only once both swings are this close to their goal
LOWEST_PUSH = 0.3  # the height loop never asks for less than this


@dataclass
class TurnHop:
    """One steered bound hop: how far, and how much the body should turn (+ = left)."""
    distance_cm: float
    turn_deg: float


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class BunnyController(BaselineController):
    def __init__(self, design, front_stance_angle=None):
        super().__init__(design, front_stance_angle)
        self.bound_model = fit_model(design)  # None if no bound map (bound_map.py): no steering
        self.bound_pushes = possible_pushes(design, BOUND_LOWEST_PUSH)
        swing_range = getattr(design, "front_swing_range_deg", (0.0, 0.0))
        self.swing_range = tuple(math.radians(a) for a in swing_range)
        self.swing_launch = math.radians(SWING_LAUNCH_DEG)  # speed loop's output
        self.swing_base = self.swing_launch  # rate-limited base the attitude law adds to
        self.swing_goals = {s: self.swing_launch for s in SIDES}
        self.raibert_push = None  # height loop's output (None until the first hop of a bound)

    # --- 2. Steered bounding ---

    def steering(self):
        return self.bound is not None and self.bound.heading_deg is not None and self.bound_model is not None

    def turn_wanted(self):
        yaw = math.degrees(self.estimator.yaw())
        return float(np.clip(wrap_deg(self.bound.heading_deg - yaw), -MAX_TURN_DEG, MAX_TURN_DEG))

    def bound_hop(self):
        if self.bound is not None and self.bound.height_cm is not None:
            if self.raibert_push is None or self.bound_hops_done == 0:
                self.raibert_push = self.bound.push
            return HopCommand(self.raibert_push, self.raibert_push, 0.0)  # 3a: the height loop sets the push
        if not self.steering():
            return super().bound_hop()
        distance = self.bound.distance_cm or self.bound_model.predict(self.bound.push, self.bound.push, 0)[0]
        return TurnHop(distance, self.turn_wanted())

    def retime_hop(self, plan: HopPlan) -> HopPlan:
        if not self.steering():
            return plan
        # Decide late: the teeth are wound, but the release timing can still use the yaw right now
        wound = (self.legs["left"].energy(), self.legs["right"].energy())
        _, _, delay = best_command(self.bound_model, None, 0.0, self.turn_wanted(), fixed=wound, distance_weight=0.0)
        return dataclasses.replace(plan, right_delay=delay)

    def plan_hop(self, command: HopCommand) -> HopPlan:
        if isinstance(command, TurnHop):
            push_left, push_right, delay = best_command(self.bound_model, self.bound_pushes, command.distance_cm,
                                                        command.turn_deg)
            return HopPlan(push_left, push_right, delay, front_push=1.0, front_delay=-FRONT_LEAD)
        # 1. Front legs push first
        return HopPlan(command.push_left, command.push_right, command.right_delay, front_push=1.0,
                       front_delay=-FRONT_LEAD)

    # --- 3. Raibert-style control ---

    def in_flight(self):
        r = self.record
        return r is not None and not math.isnan(r.liftoff_time) and math.isnan(r.touchdown_time)

    def landing(self):
        r = self.record
        return r is not None and not math.isnan(r.touchdown_time)

    def front_swing_goals(self, inputs: Inputs) -> dict:
        lo, hi = self.swing_range
        if self.in_flight():
            # Place the front feet for the landing; the base starts the landing from there
            self.swing_base = math.radians(SWING_TOUCHDOWN_DEG)
            goal = self.swing_base
        else:
            if not self.landing():  # winding, settling, pushing: walk the feet to the launch placement
                step = SWING_SLEW * DT
                self.swing_base += float(np.clip(self.swing_launch - self.swing_base, -step, step))
            goal = self.swing_base
            if self.landing() or ATTITUDE_WHEN == "stance":
                # 3c. Attitude: torque the body level through the planted front feet
                _, pitch = self.estimator.roll_pitch()
                goal += ATTITUDE_KP * pitch + ATTITUDE_KD * self.estimator.gyro[1]
        goal = float(np.clip(goal, lo, hi))
        self.swing_goals = {s: goal for s in SIDES}
        return {s: ServoCommand(goal) for s in SIDES}

    def ready_to_launch(self, inputs: Inputs, bounding: bool) -> bool:
        if self.design.front_swing:
            on_base = abs(self.swing_base - self.swing_launch) < 1e-6
            placed = all(inputs.servo_position.get(f"front_swing_{s}") is not None
                         and abs(inputs.servo_position[f"front_swing_{s}"] - self.swing_goals[s]) < SWING_READY
                         for s in SIDES)
            if not (on_base and placed) and self.time_in_status < 1.0:
                return False
        return super().ready_to_launch(inputs, bounding)

    def on_hop_finished(self, record: HopRecord):
        if self.bound is None:
            return
        # 3a. Height: correct the next push toward the target height
        if self.bound.height_cm is not None and self.raibert_push is not None and not math.isnan(record.flight_time):
            error = self.bound.height_cm / 100 - record.hop_height
            self.raibert_push = float(np.clip(self.raibert_push + HEIGHT_GAIN * error, LOWEST_PUSH, 1.0))
        # 3b. Speed: feet further forward for the next launch if it went too slow
        if self.design.front_swing and self.bound.speed_cm_s is not None and not math.isnan(record.liftoff_speed):
            error = self.bound.speed_cm_s / 100 - record.liftoff_speed
            lo, hi = (math.radians(a) for a in SPEED_SWING_RANGE_DEG)
            self.swing_launch = float(np.clip(self.swing_launch + SPEED_GAIN * error, lo, hi))
