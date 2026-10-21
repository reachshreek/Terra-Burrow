"""Per-leg logic: what each leg does in each phase of a hop.

LatchLegLogic   rear leg, wind-and-latch: wind (or unwind) to a ratchet tooth,
                trip the sear to fire, re-arm and reel back in during flight
FrontLegLogic   servo front legs: push at takeoff, tuck, reach before landing,
                then level the body with a roll/pitch PD loop on the IMU
"""

import math

import numpy as np

from .interface import DT, SIDES, ServoCommand

TUCK_TOLERANCE = math.radians(3)  # a push is "done" within this of its end angle
MAX_PUSH_TIME = 0.3  # s, a leg that never reaches its push angle tucks after this
RELEASE_WAIT = 0.06  # s; the release servo needs up to a PWM frame plus its travel time to move
# The winder's position loop is proportional only, so against the spring it stops short of its goal
# (about 2° at full wind). Aim this far past where the leg needs to get.
CORD_OVERSHOOT_TEETH = 1.5


class LatchLegLogic:
    """Wind-and-latch rear leg.

    The winder servo reels a cord that pulls the leg into the crouch, stretching
    the spring. A ratchet pawl drops into a tooth every few degrees and holds the
    leg; the tooth it's on sets the hop energy. Tripping the sear lifts the pawl
    and the spring fires the leg.

    The ratchet only holds against extension, so moving to a LOWER tooth means:
    take the load on the cord, trip the sear, pay the cord out past the tooth,
    re-arm, then reel back up onto it.

    Phases:
      reel_in   cord pulls the leg up just past the target tooth
      seat      cord goes slack; the pawl takes the load on the tooth
      unload    (going down) cord pulls a little to take the load off the pawl
      lift      (going down) trip the sear, with the cord still holding
      pay_out   (going down) let the cord out to just below the target tooth
      rearm     (going down) re-arm the sear, then reel_in
      latched   ready to fire
      firing    sear tripped, cord slack: the spring pushes the leg out
      tucking   sear re-armed, cord reels the leg back up to the crouch for landing
      holding   tucked and resting on the pawl
    """

    def __init__(self, design, name):
        self.name = name
        self.crouch = math.radians(design.crouch_angle_deg)
        self.slack = math.radians(design.latch_spring_slack_angle_deg)
        self.full_push = math.radians(design.full_push_angle_deg)
        self.pitch = math.radians(design.ratchet_tooth_pitch_deg)
        self.target = self.crouch
        self.phase = "holding"
        self.time = 0.0
        self.fired = False
        self.goal = self.crouch
        self.torque_on = False
        self.release = False

    # --- ratchet geometry: teeth every `pitch` down from the crouch ---

    def energy(self, tooth=None):
        """Fraction of full-wind energy the spring stores on `tooth` (default: the target)."""
        tooth = self.target if tooth is None else tooth
        return ((tooth - self.slack) / (self.crouch - self.slack)) ** 2

    def tooth_for(self, push):
        """The tooth that stores closest to `push` of full-wind energy (energy ~ stretch²)."""
        angle = self.slack + math.sqrt(max(push, 0.0)) * (self.crouch - self.slack)
        teeth = round((self.crouch - angle) / self.pitch)
        return self.crouch - teeth * self.pitch

    def tooth_below(self, angle):
        """The tooth the pawl would catch on if the leg dropped from `angle`."""
        teeth = math.ceil((self.crouch - angle) / self.pitch - 0.02)  # within 2% of a tooth counts as on it
        return self.crouch - max(teeth, 0) * self.pitch

    # --- interface ---

    def prepare(self, push):
        self.target = self.tooth_for(push)
        self.fired = False
        self.set_phase("plan")

    def ready(self, position):
        return self.phase == "latched"

    def fire(self):
        self.fired = True
        self.set_phase("firing")

    def done(self):
        return self.fired and self.phase in ("holding",)

    def set_phase(self, phase):
        self.phase = phase
        self.time = 0.0

    def tick(self, position):
        self.time += DT
        if position is None:
            return ServoCommand(self.goal, False), self.release
        half = 0.5 * self.pitch

        if self.phase == "plan":
            on = self.tooth_below(position + 0.3 * self.pitch)
            if abs(on - self.target) < 0.1 * self.pitch:
                self.set_phase("latched")
            elif on < self.target:
                self.set_phase("reel_in")
            else:
                self.set_phase("unload")

        if self.phase == "reel_in":
            self.goal, self.torque_on, self.release = self.target + CORD_OVERSHOOT_TEETH * self.pitch, True, False
            if position > self.target + 0.3 * self.pitch or self.time > 2.0:
                self.set_phase("seat")
        elif self.phase == "seat":
            self.torque_on = False
            if self.time > 0.05:
                self.set_phase("plan")  # check which tooth it landed on
        elif self.phase == "unload":
            if self.time <= DT:
                self.unload_tooth = self.tooth_below(position + 0.3 * self.pitch)
            self.goal, self.torque_on = self.unload_tooth + CORD_OVERSHOOT_TEETH * self.pitch, True
            if position > self.unload_tooth + 0.2 * self.pitch or self.time > 0.5:
                self.set_phase("lift")
        elif self.phase == "lift":
            self.release = True
            if self.time > RELEASE_WAIT:
                self.set_phase("pay_out")
        elif self.phase == "pay_out":
            self.goal = self.target - half
            if position < self.target - 0.3 * self.pitch or self.time > 1.0:
                self.set_phase("rearm")
        elif self.phase == "rearm":
            self.release = False
            if self.time > RELEASE_WAIT:
                self.set_phase("reel_in")
        elif self.phase == "latched":
            self.torque_on, self.release = False, False
        elif self.phase == "firing":
            self.torque_on, self.release = False, True
            if position < self.full_push + TUCK_TOLERANCE or self.time > MAX_PUSH_TIME:
                self.set_phase("tucking")
        elif self.phase == "tucking":
            # Aim at the crouch and stop within TUCK_TOLERANCE, resting on whatever tooth that is
            self.goal, self.torque_on, self.release = self.crouch, True, False
            if position > self.crouch - TUCK_TOLERANCE or self.time > 1.0:
                self.set_phase("holding")
        elif self.phase == "holding":
            self.torque_on, self.release = False, False
        return ServoCommand(self.goal, self.torque_on), self.release


# ---------------------------------------------------------------------------
# Front legs (servo-driven designs only)
# ---------------------------------------------------------------------------

MAX_FRONT_PUSH_TIME = 0.15  # s
TUCK_ANGLE_DEG = 70.0  # fold this far in flight so the feet clear the floor
REACH_DEG = 12.0  # before landing, reach this much straighter than stance, to have travel to absorb with
FRONT_TUCK_TIME = 0.06  # s in the tuck before reaching for the floor
FREE_FALL_BELOW_G = 0.3  # touchdown = free fall, then a spike
TOUCHDOWN_ABOVE_G = 1.8
MAX_REACH_TIME = 0.6  # s

PITCH_KP = 0.12  # m of leg length per rad of nose-down pitch
PITCH_KD = 0.006  # m per rad/s
ROLL_KP = 0.10  # m per rad of roll (high side shortens, low side lengthens)
ROLL_KD = 0.005  # m per rad/s
MAX_LENGTH_CHANGE = 0.025  # m


class FrontLegLogic:
    """Both servo front legs: push, tuck, reach, then leveled stance."""

    def __init__(self, design, stance_angle):
        self.leg_length = design.front_thigh_length + design.front_shin_length
        self.stance = stance_angle
        self.full_push = math.radians(design.front_full_push_angle_deg)
        self.hip_range = tuple(math.radians(a) for a in design.front_hip_range_deg)
        self.tuck_angle = math.radians(TUCK_ANGLE_DEG)
        self.reach_angle = self.stance - math.radians(REACH_DEG)
        self.phase = "stance"
        self.time = 0.0
        self.time_to_push = None
        self.push = 1.0
        self.felt_free_fall = False

    def fire(self, delay=0.0, push=1.0):
        """Push after `delay` s, straightening `push` of the way (0 = stay at stance, 1 = fully)."""
        self.time_to_push = delay
        self.push = push

    def set_phase(self, phase):
        self.phase = phase
        self.time = 0.0
        self.felt_free_fall = False

    def length_to_angle(self, length):
        return math.acos(np.clip(length / self.leg_length, 0.0, 1.0))

    def leveled(self, estimator):
        if not estimator.ready():
            return {s: self.stance for s in SIDES}
        roll, pitch = estimator.roll_pitch()
        roll_rate, pitch_rate = estimator.gyro[0], estimator.gyro[1]
        base = self.leg_length * math.cos(self.stance)
        pitch_change = PITCH_KP * pitch + PITCH_KD * pitch_rate
        roll_change = ROLL_KP * roll + ROLL_KD * roll_rate
        out = {}
        for side, sign in (("left", -1), ("right", +1)):
            change = np.clip(pitch_change + sign * roll_change, -MAX_LENGTH_CHANGE, MAX_LENGTH_CHANGE)
            out[side] = self.length_to_angle(base + change)
        return out

    def tick(self, estimator, positions):
        self.time += DT
        if self.time_to_push is not None:
            self.time_to_push -= DT
            if self.time_to_push <= 0:
                self.time_to_push = None
                self.set_phase("push")

        if self.phase == "push":
            push_to = self.stance - self.push * (self.stance - self.full_push)
            goals = {s: push_to for s in SIDES}
            straight = all(p is not None and abs(p - push_to) < math.radians(3) for p in positions.values())
            if straight or self.time > MAX_FRONT_PUSH_TIME:
                self.set_phase("tuck")
        elif self.phase == "tuck":
            goals = {s: self.tuck_angle for s in SIDES}
            if self.time > FRONT_TUCK_TIME:
                self.set_phase("reach")
        elif self.phase == "reach":
            # Leveling in the air would land it on one stretched leg, so hold still until touchdown
            goals = {s: self.reach_angle for s in SIDES}
            g = estimator.g_felt()
            self.felt_free_fall |= g < FREE_FALL_BELOW_G
            if (self.felt_free_fall and g > TOUCHDOWN_ABOVE_G) or self.time > MAX_REACH_TIME:
                self.set_phase("stance")
        else:
            goals = self.leveled(estimator)
        return {s: ServoCommand(float(np.clip(goals[s], *self.hip_range))) for s in SIDES}
