"""A simple model of a position servo such as the Waveshare ST3025.

Inside the servo, a position controller asks for some torque. The motor can
only deliver what a DC motor at full voltage allows, which is the
torque-speed line:

    max torque = stall torque * (1 - speed / no-load speed)

So the faster the servo is already turning, the less torque it can add. At
no-load speed it can't push any harder. This is the main thing limiting how
hard the bunny can hop.

The position controller only knows what the servo's own 12-bit magnetic
encoder tells it. It runs at a fixed rate, estimates speed by differencing
encoder counts (then low-pass filtering), ignores errors inside its deadband,
and holds its torque between updates. The host only sees the position the
servo reports back over the bus, at the host's loop rate.
"""

import math
from dataclasses import dataclass, field

import numpy as np

ENCODER_STEP = 2 * math.pi / 4096  # rad per count at the servo output
INTERNAL_LOOP_HZ = 1000.0  # servo firmware position loop (guess; Feetech doesn't publish it)
DEADBAND_COUNTS = 1  # STS-series default
SPEED_FILTER = 0.3  # low-pass on the differenced speed, per internal loop (0-1, higher = less smoothing)
BUS_READ_HZ = 200.0  # how often the host reads each servo's position back


@dataclass
class Servo:
    stall_torque: float  # N·m at the servo output
    no_load_speed: float  # rad/s at the servo output
    stall_current: float  # A
    kp: float  # N·m per rad of position error
    kd: float  # N·m per rad/s
    linkage_ratio: float = 1.0  # hip rad per servo rad
    last_current: float = 0.0  # A, set by hip_torque()
    reported_hip_angle: float | None = None  # rad, what the host last read back over the bus

    _loop_timer: float = field(default=0.0, repr=False)
    _read_timer: float = field(default=0.0, repr=False)
    _last_counts: int | None = field(default=None, repr=False)
    _speed_estimate: float = field(default=0.0, repr=False)
    _requested: float = field(default=0.0, repr=False)

    def hip_torque(self, target_hip_angle, hip_angle, hip_speed, dt):
        """Torque applied at the hip joint, given where we want it to be.

        hip_angle and hip_speed are the true joint state: the angle only reaches
        the controller through the encoder, and the speed only sets the motor's
        back-EMF (the physics), never the control law.
        """
        counts = round(hip_angle / self.linkage_ratio / ENCODER_STEP)

        self._read_timer -= dt
        if self._read_timer <= 0 or self.reported_hip_angle is None:
            self._read_timer += 1.0 / BUS_READ_HZ
            self.reported_hip_angle = counts * ENCODER_STEP * self.linkage_ratio

        self._loop_timer -= dt
        if self._loop_timer <= 0 or self._last_counts is None:
            period = 1.0 / INTERNAL_LOOP_HZ
            self._loop_timer += period
            if self._last_counts is not None:
                raw_speed = (counts - self._last_counts) * ENCODER_STEP / period
                self._speed_estimate += SPEED_FILTER * (raw_speed - self._speed_estimate)
            self._last_counts = counts
            target_counts = round(target_hip_angle / self.linkage_ratio / ENCODER_STEP)
            error_counts = target_counts - counts
            if abs(error_counts) <= DEADBAND_COUNTS:
                error_counts = 0
            self._requested = self.kp * error_counts * ENCODER_STEP - self.kd * self._speed_estimate

        # Full forward or full reverse voltage, minus back-EMF from the current speed
        servo_speed = hip_speed / self.linkage_ratio
        most_positive = self.stall_torque * (1 - servo_speed / self.no_load_speed)
        most_negative = self.stall_torque * (-1 - servo_speed / self.no_load_speed)
        servo_torque = np.clip(self._requested, most_negative, most_positive)
        # The driver's current limit caps torque at the stall value either way
        servo_torque = np.clip(servo_torque, -self.stall_torque, self.stall_torque)

        self.last_current = abs(servo_torque) / self.stall_torque * self.stall_current
        return servo_torque / self.linkage_ratio

    def spec_copy(self, linkage_ratio=None):
        """The same servo model, fresh (no encoder history), optionally through different gearing."""
        return Servo(self.stall_torque, self.no_load_speed, self.stall_current, self.kp, self.kd,
                     self.linkage_ratio if linkage_ratio is None else linkage_ratio)


def servo_from_design(design, strength_scale=1.0, speed_scale=1.0, linkage_ratio=None):
    return Servo(
        stall_torque=design.servo_stall_torque * strength_scale,
        no_load_speed=design.servo_no_load_speed * speed_scale,
        stall_current=design.servo_stall_current,
        kp=design.servo_kp,
        kd=design.servo_kd,
        linkage_ratio=1.0 if linkage_ratio is None else linkage_ratio,
    )
