"""The robot's wiring and calibration sheet: everything the firmware needs to turn raw numbers into physics.

On the real robot you'd fill this in once per robot, at assembly: which bus ID
is which servo, the encoder tick where each joint reads zero, which way each
horn faces. Here it matches how the sim's robot is assembled (sim_hal.ASSEMBLY).
"""

import math

from build_model import front_stance_angle
from config import DESIGNS
from legs import winder_gear_ratio

DESIGN = "latch_tethered_front"  # which robot this firmware is built for (run_firmware.py --design sets it)
CONTROL_LAWS = True  # False: BaselineController, without bunny_controller.py's laws (run_firmware.py --no-controller)

CONTROL_RATE_HZ = 200
TICK_US = 1_000_000 // CONTROL_RATE_HZ

# --- MPU-6050 (I2C 0x68), set up as ±16 g and ±2000 °/s ---
ACCEL_COUNTS_PER_G = 2048.0
GYRO_COUNTS_PER_DPS = 16.4
ACCEL_OFFSET = (0.0, 0.0, 0.0)  # counts; from a six-face bench calibration (not done yet)
GYRO_CAL_SAMPLES = 100  # averaged at boot, one per tick (0.5 s); the robot must sit still
GYRO_CAL_MAX_SPREAD = 40  # counts (~2.4 °/s): if any axis wanders more than this, it moved: try again

# --- ST3025 bus servos: bus ID, encoder ticks at joint angle 0, horn direction ---
SERVOS = {
    "rear_left": (1, 512, +1),
    "rear_right": (2, 3584, -1),
    "front_left": (3, 1024, +1),
    "front_right": (4, 3072, -1),
}
TICKS_PER_REV = 4096

# --- Release servos (MG90D) on the PCA9685 ---
RELEASE_CHANNEL = {"left": 0, "right": 1}
RELEASE_ARMED_US = 1500
RELEASE_TRIP_US = 1550

# --- Rear foot switches: GPIO, internal pull-up, switch to ground ---
FOOT_PIN = {"left": 14, "right": 15}
FOOT_DEBOUNCE_TICKS = 2  # a change must read the same this many ticks in a row


def design():
    return DESIGNS[DESIGN]


def front_stance():
    return front_stance_angle(design())


def servo_turns_per_joint_turn(name):
    """Servo shaft radians per joint radian: the winder's gearing on the rear legs, direct on the front."""
    return winder_gear_ratio(design()) if name.startswith("rear") else 1.0


def servo_names():
    return list(SERVOS)


RAD_PER_TICK = 2 * math.pi / TICKS_PER_REV
