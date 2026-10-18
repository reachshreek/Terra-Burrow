"""Rear-leg drivers: how each hind leg is actuated during a hop.

Both drivers have the same interface, which simulate_hop() in hop.py uses:
    prepare(push)   get ready for a hop with this much push (0-1)
    is_ready()      True once the leg is ready to fire
    fire()          start the push
    is_tucked()     True once the push is over and the leg is folding back
    update(angle, speed, dt) -> (servo hip torque, spring hip torque)

ServoLeg  - the BOM as it stands: the servo swings the leg directly.
LatchLeg  - wind-and-latch: the servo winds the leg into the crouch with a cord,
            stretching a stiff spring; a ratchet pawl holds it; releasing the pawl
            lets the spring snap the leg out at its own speed.
"""

import math

from servo import Servo, servo_from_design

TUCK_TOLERANCE = math.radians(3)  # a push is "done" within this of its end angle
MAX_PUSH_TIME = 0.3  # s, a leg that never reaches its push angle tucks after this
LATCH_TOLERANCE = math.radians(1)  # a leg already this close to its tooth doesn't rewind
WIND_OVERSHOOT = math.radians(3)  # the cord pulls this far past the tooth so the pawl clicks in


def push_angle(design, push):
    """Hip angle for a push fraction: 0 = crouched, 1 = fully extended."""
    crouch = math.radians(design.crouch_angle_deg)
    full = math.radians(design.full_push_angle_deg)
    return crouch - push * (crouch - full)


class ServoLeg:
    def __init__(self, design, servo: Servo):
        self.servo = servo
        self.design = design
        self.crouch = math.radians(design.crouch_angle_deg)
        self.target = self.crouch
        self.phase = "crouched"  # crouched -> pushing -> tucked
        self.time_pushing = 0.0
        self.pending_push = 1.0
        self.current = 0.0  # A, servo current on the last step

    def prepare(self, push):
        self.pending_push = push
        self.target = self.crouch
        self.phase = "crouched"

    def is_ready(self):
        return True

    def fire(self):
        self.phase = "pushing"
        self.time_pushing = 0.0
        self.target = push_angle(self.design, self.pending_push)

    def is_tucked(self):
        return self.phase == "tucked"

    def update(self, angle, speed, dt):
        if self.phase == "pushing":
            self.time_pushing += dt
            # Like the real bus servo: tuck once it reports reaching the push angle
            if abs(angle - self.target) < TUCK_TOLERANCE or self.time_pushing > MAX_PUSH_TIME:
                self.phase = "tucked"
                self.target = self.crouch
        torque = self.servo.hip_torque(self.target, angle, speed)
        self.current = self.servo.last_current
        return torque, 0.0


class LatchLeg:
    """Wind-and-latch hind leg.

    Phases:
      winding   the servo reels in the cord, pulling the leg toward its tooth
      latched   the pawl holds the leg; the cord is slack
      released  the pawl is tripped; only the spring acts on the leg
      tucking   after the push, the cord pulls the leg back toward the crouch
    """

    def __init__(self, design, servo: Servo):
        self.design = design
        # Through the winder gears the servo is slower but stronger at the hip
        self.winder = Servo(
            stall_torque=servo.stall_torque,
            no_load_speed=servo.no_load_speed,
            stall_current=servo.stall_current,
            kp=servo.kp,
            kd=servo.kd,
            linkage_ratio=1.0 / winder_gear_ratio(design),
        )
        self.stiffness = design.latch_spring_stiffness
        self.slack_angle = math.radians(design.latch_spring_slack_angle_deg)
        self.crouch = math.radians(design.crouch_angle_deg)
        self.full_push = math.radians(design.full_push_angle_deg)

        self.phase = "latched"
        self.latch_angle = self.crouch  # the keyframe starts fully wound and latched
        self.wind_target = self.crouch
        self.time_released = 0.0
        self.wind_time = 0.0  # how long the last winding took
        self.current = 0.0  # A, winder servo current on the last step

    def tooth_angle(self, push):
        """The ratchet tooth that stores `push` of the full-wind energy.

        Spring energy grows with the square of the stretch, so the stretch is
        the square root of the energy fraction.
        """
        return self.slack_angle + math.sqrt(push) * (self.crouch - self.slack_angle)

    def prepare(self, push):
        target = self.tooth_angle(push)
        if self.phase == "latched" and abs(self.latch_angle - target) < LATCH_TOLERANCE:
            return  # already on the right tooth
        self.phase = "winding"
        self.wind_target = target
        self.wind_direction = None  # +1 reeling in toward the crouch, -1 letting out
        self.wind_time = 0.0

    def is_ready(self):
        return self.phase == "latched"

    def fire(self):
        self.phase = "released"
        self.time_released = 0.0

    def is_tucked(self):
        return self.phase in ("tucking", "winding", "latched") and self.time_released > 0

    @property
    def latched(self):
        return self.phase == "latched"

    def spring_torque(self, angle):
        """Pushes toward extension (smaller hip angle) whenever the spring is stretched."""
        stretch = max(0.0, angle - self.slack_angle)
        return -self.stiffness * stretch

    def cord_torque(self, target, angle, speed):
        """The cord can only pull the leg toward the crouch (positive hip torque)."""
        torque = max(0.0, self.winder.hip_torque(target, angle, speed))
        servo_torque = torque * self.winder.linkage_ratio
        self.current = servo_torque / self.winder.stall_torque * self.winder.stall_current
        return torque

    def update(self, angle, speed, dt):
        spring = self.spring_torque(angle)
        self.current = 0.0

        if self.phase == "winding":
            self.wind_time += dt
            if self.wind_direction is None:
                self.wind_direction = 1 if angle < self.wind_target else -1
            # Like a ratchet: the pawl drops in as the leg passes its tooth
            passed_tooth = (angle - self.wind_target) * self.wind_direction >= 0
            if passed_tooth:
                self.phase = "latched"
                self.latch_angle = self.wind_target
                return 0.0, spring
            pull_to = self.wind_target + self.wind_direction * WIND_OVERSHOOT
            return self.cord_torque(pull_to, angle, speed), spring

        if self.phase == "latched":
            return 0.0, spring  # the pawl (an equality constraint) holds the leg

        if self.phase == "released":
            self.time_released += dt
            if angle < self.full_push + TUCK_TOLERANCE or self.time_released > MAX_PUSH_TIME:
                self.phase = "tucking"
            return 0.0, spring

        # tucking: pull the leg back up for landing; the next prepare() winds it to a tooth
        return self.cord_torque(self.crouch, angle, speed), spring


# ---------------------------------------------------------------------------
# Sizing helpers for the wind-and-latch mechanism
# ---------------------------------------------------------------------------

def peak_spring_torque(design):
    """Hip torque from the spring when fully wound into the crouch."""
    stretch = math.radians(design.crouch_angle_deg - design.latch_spring_slack_angle_deg)
    return design.latch_spring_stiffness * stretch


def winder_gear_ratio(design):
    """Servo turns per hip turn, chosen so full wind needs only a fraction of stall torque."""
    usable_servo_torque = design.winder_torque_fraction * design.servo_stall_torque
    return max(1.0, peak_spring_torque(design) / usable_servo_torque)


def winding_report(design, push=1.0):
    """Energy, forces and wind-up time for one leg at a given push."""
    gear = winder_gear_ratio(design)
    k = design.latch_spring_stiffness
    full_stretch = math.radians(design.crouch_angle_deg - design.latch_spring_slack_angle_deg)
    stretch = math.sqrt(push) * full_stretch  # push is the fraction of full-wind energy

    # Wind from slack to full stretch. At each step the servo's speed drops as
    # the spring pushes back harder (the torque-speed line).
    steps = 400
    wind_time = 0.0
    for i in range(steps):
        hip_angle_step = stretch / steps
        hip_torque = k * (i + 0.5) * hip_angle_step
        servo_torque = hip_torque / gear
        servo_speed = design.servo_no_load_speed * (1 - servo_torque / design.servo_stall_torque)
        wind_time += hip_angle_step * gear / servo_speed

    hip_torque = k * stretch
    return {
        "energy_j": 0.5 * k * stretch**2,
        "peak_hip_torque_nm": hip_torque,
        "pawl_force_n": hip_torque / design.latch_lever_radius,
        "winder_gear_ratio": gear,
        "peak_servo_torque_nm": hip_torque / gear,
        "wind_time_s": wind_time,
    }


def make_leg(design, servo=None):
    servo = servo or servo_from_design(design)
    return LatchLeg(design, servo) if design.drive == "latch" else ServoLeg(design, servo)
