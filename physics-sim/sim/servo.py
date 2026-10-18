"""A simple model of a position servo such as the Waveshare ST3025.

Inside the servo, a position controller asks for some torque. The motor can
only deliver what a DC motor at full voltage allows, which is the
torque-speed line:

    max torque = stall torque * (1 - speed / no-load speed)

So the faster the servo is already turning, the less torque it can add. At
no-load speed it can't push any harder. This is the main thing limiting how
hard the bunny can hop.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Servo:
    stall_torque: float  # N·m at the servo output
    no_load_speed: float  # rad/s at the servo output
    stall_current: float  # A
    kp: float  # N·m per rad of position error
    kd: float  # N·m per rad/s
    linkage_ratio: float = 1.0  # hip rad per servo rad
    last_current: float = 0.0  # A, set by hip_torque()

    def hip_torque(self, target_hip_angle, hip_angle, hip_speed):
        """Torque applied at the hip joint, given where we want it to be."""
        servo_error = (target_hip_angle - hip_angle) / self.linkage_ratio
        servo_speed = hip_speed / self.linkage_ratio

        requested = self.kp * servo_error - self.kd * servo_speed

        # Full forward or full reverse voltage, minus back-EMF from the current speed
        most_positive = self.stall_torque * (1 - servo_speed / self.no_load_speed)
        most_negative = self.stall_torque * (-1 - servo_speed / self.no_load_speed)
        servo_torque = np.clip(requested, most_negative, most_positive)
        # The driver's current limit caps torque at the stall value either way
        servo_torque = np.clip(servo_torque, -self.stall_torque, self.stall_torque)

        self.last_current = abs(servo_torque) / self.stall_torque * self.stall_current
        return servo_torque / self.linkage_ratio


def servo_from_design(design, strength_scale=1.0, speed_scale=1.0):
    return Servo(
        stall_torque=design.servo_stall_torque * strength_scale,
        no_load_speed=design.servo_no_load_speed * speed_scale,
        stall_current=design.servo_stall_current,
        kp=design.servo_kp,
        kd=design.servo_kd,
        linkage_ratio=design.linkage_ratio,
    )
