"""The bunny's controller: bigger, continuous, steerable hops.

Built on the machinery in controller/ (estimator, leg phases, front-leg leveling,
sequencing). Three control laws, each kept because it measurably helped in the
sim (see README, "Controller", and controller_eval.py):

1. Front-lead feedforward. The front legs push FRONT_LEAD before the rear legs
   are released, lifting the nose first so the rear push doesn't throw the body
   into a spin it can't land from.

2. Bounding ramp. Each run of bounding hops starts at BOUND_RAMP_START of the
   commanded push and ramps up over BOUND_RAMP_HOPS hops, so the body settles
   into a steady rhythm before the hops get big.

3. Steered bounding (BoundCommand.heading_deg). A heading loop, one correction
   per hop: turn = clip(heading - yaw, ±MAX_TURN_DEG). At each landing the teeth
   and release timing are picked from a model fitted on bounding hops
   (bound_map.py); once the teeth are wound, the release timing is picked again
   at the moment of launch, from the yaw measured then.
"""

import dataclasses
from dataclasses import dataclass
import math

import numpy as np

from controller import BaselineController, HopCommand, HopPlan, HopRecord, Inputs
from controller.bound_model import best_command, fit_model, possible_pushes

# --- 1. Front-lead feedforward ---
# Step test (16 robots, Pico timers): upright with a 0 / 15 / 30 ms lead was 100/100/100% at
# 7 N·m/rad, 100/100/94% at 10, and 31/94/75% at 13. Costs ~20% of distance at 13.
FRONT_LEAD = 0.05  # s

# --- 3. Steered bounding ---
MAX_TURN_DEG = 20.0  # most body turn asked of one bound hop
BOUND_LOWEST_PUSH = 0.5  # the bound map covers teeth from here up

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

    def ramp(self):
        return 1

    def steering(self):
        return self.bound is not None and self.bound.heading_deg is not None and self.bound_model is not None

    def turn_wanted(self):
        yaw = math.degrees(self.estimator.yaw())
        return float(np.clip(wrap_deg(self.bound.heading_deg - yaw), -MAX_TURN_DEG, MAX_TURN_DEG))

    def bound_hop(self):
        if not self.steering():
            return super().bound_hop()
        # 3. Steered bounding: turn toward the heading, at most MAX_TURN_DEG a hop
        distance = self.bound.distance_cm or self.bound_model.predict(self.bound.push, self.bound.push, 0)[0]
        return TurnHop(distance * self.ramp(), self.turn_wanted())

    def retime_hop(self, plan: HopPlan) -> HopPlan:
        if not self.steering():
            return plan
        # 3. Decide late: the teeth are wound, but the release timing can still use the yaw right now
        wound = (self.legs["left"].energy(), self.legs["right"].energy())
        _, _, delay = best_command(self.bound_model, None, 0.0, self.turn_wanted(), fixed=wound, distance_weight=0.0)
        return dataclasses.replace(plan, right_delay=delay)

    def plan_hop(self, command: HopCommand) -> HopPlan:
        if isinstance(command, TurnHop):
            push_left, push_right, delay = best_command(self.bound_model, self.bound_pushes, command.distance_cm,
                                                        command.turn_deg)
            return HopPlan(push_left, push_right, delay, front_push=1.0, front_delay=-FRONT_LEAD)
        push_left, push_right, delay = command.push_left, command.push_right, command.right_delay

        # 2. Bounding ramp
        if self.bound is not None:
            push_left, push_right = push_left * self.ramp(), push_right * self.ramp()

        # 1. Front legs push first
        return HopPlan(push_left, push_right, delay, front_push=1.0, front_delay=-FRONT_LEAD)

    def ready_to_launch(self, inputs: Inputs, bounding: bool) -> bool:
        return super().ready_to_launch(inputs, bounding)

    def front_leg_goals(self, goals: dict, inputs: Inputs) -> dict:
        return goals

    def on_hop_finished(self, record: HopRecord):
        pass
