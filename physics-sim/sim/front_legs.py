"""Servo-driven front legs and the balance controller that runs them.

Only for designs with front_drive = "servo". Each front leg is a thigh + shin
with a coupled knee (like the rear legs) swung by an ST3025, hanging under the
sprung strut. The controller does two jobs:

  push    when the rear legs fire, the front legs straighten too, lifting the
          nose so the hop leaves the ground level instead of nose-down
  level   once the IMU feels the landing, and while standing, each front leg
          lengthens or shortens to hold the body level: a PD loop on roll and
          pitch from the IMU. This is what catches a crooked landing.

The controller sees only what the robot would: its own servo encoders and the
MPU-6050 (HAL2-008), read through a noise, bias and range model, at a fixed
loop rate, with each servo command arriving one loop late. It never reads the
simulator's true body pose.
"""

import math

import mujoco
import numpy as np

from build_model import front_stance_angle
from servo import servo_from_design

SIDES = ("left", "right")
G = 9.81

# --- Controller loop (on the Pico 2, HAL2-007) ---
CONTROL_RATE_HZ = 200.0
ENCODER_STEP = 2 * math.pi / 4096  # rad, the ST3025's 12-bit position readback

# --- Push at takeoff ---
FRONT_PUSH_DELAY = 0.0  # s after the first rear leg fires
FRONT_PUSH = 1.0  # 0 = stay at stance, 1 = straighten to full extension
MAX_FRONT_PUSH_TIME = 0.15  # s, stop pushing after this even if not straight yet
TUCK_ANGLE_DEG = 70.0  # fold the front legs this far in flight so the feet clear the floor
REACH_DEG = 12.0  # before landing, reach this much straighter than stance, to have travel to absorb with
TUCK_TIME = 0.06  # s in the tuck before reaching for the floor
# Touchdown, from the accelerometer: free fall reads near 0 g, the landing spikes well above 1 g
FREE_FALL_BELOW_G = 0.3
TOUCHDOWN_ABOVE_G = 1.8
MAX_REACH_TIME = 0.6  # s, start leveling after this even if no touchdown was felt

# --- Leveling: leg-length change per unit of attitude error ---
PITCH_KP = 0.12  # m per rad of nose-down pitch (a nose-down body gets longer front legs)
PITCH_KD = 0.006  # m per rad/s
ROLL_KP = 0.10  # m per rad of roll (the high side's leg shortens, the low side's lengthens)
ROLL_KD = 0.005  # m per rad/s
MAX_LENGTH_CHANGE = 0.025  # m, leveling never asks for more than this either way

# --- Attitude filter (Mahony complementary filter) ---
FILTER_KP = 2.0  # rad/s per unit of gravity-direction error
TRUST_ACCEL_WITHIN_G = 0.15  # only correct with the accelerometer when |a| is this close to 1 g
TRUST_ACCEL_AFTER_LANDING = 0.2  # s; in a hop and just after, the accelerometer isn't measuring gravity

# --- MPU-6050 at ±2000 °/s and ±16 g, after a gyro-bias calibration at power-up ---
GYRO_NOISE = math.radians(0.07)  # rad/s rms per sample
GYRO_BIAS = math.radians(0.5)  # rad/s, residual bias left after calibration (random per robot)
GYRO_RANGE = math.radians(2000)
ACCEL_NOISE = 0.055  # m/s² rms per sample
ACCEL_BIAS = 0.2  # m/s², zero-g offset left after calibration
ACCEL_RANGE = 16 * G
MOUNT_ERROR = math.radians(1.0)  # rad, how crooked the board might sit on the chassis


class Imu:
    """MPU-6050 readings: the true rates plus bias, noise, misalignment and clipping."""

    def __init__(self, model, data, rng):
        self.data = data
        self.rng = rng
        self.gyro_addr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")]
        self.accel_addr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")]
        self.gyro_bias = rng.uniform(-GYRO_BIAS, GYRO_BIAS, 3)
        self.accel_bias = rng.uniform(-ACCEL_BIAS, ACCEL_BIAS, 3)
        tilt = rng.uniform(-MOUNT_ERROR, MOUNT_ERROR, 3)
        self.mount = rotation_from_vector(tilt)

    def read(self):
        gyro = self.mount @ self.data.sensordata[self.gyro_addr:self.gyro_addr + 3]
        accel = self.mount @ self.data.sensordata[self.accel_addr:self.accel_addr + 3]
        gyro = gyro + self.gyro_bias + self.rng.normal(0, GYRO_NOISE, 3)
        accel = accel + self.accel_bias + self.rng.normal(0, ACCEL_NOISE, 3)
        return np.clip(gyro, -GYRO_RANGE, GYRO_RANGE), np.clip(accel, -ACCEL_RANGE, ACCEL_RANGE)


def rotation_from_vector(v):
    """Rotation matrix for a rotation of |v| radians about v (Rodrigues)."""
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


class AttitudeFilter:
    """Integrates the gyro; nudges toward the accelerometer's "down" when told it's measuring gravity."""

    def __init__(self, accel):
        self.R = self.level_from_accel(accel)  # body -> world

    @staticmethod
    def level_from_accel(accel):
        up = accel / np.linalg.norm(accel)  # world up, in the body frame
        roll = math.atan2(up[1], up[2])
        pitch = math.atan2(-up[0], math.hypot(up[1], up[2]))
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])

    def update(self, gyro, accel, dt, trust_accel=True):
        omega = gyro.copy()
        a = np.linalg.norm(accel)
        if trust_accel and abs(a - G) < TRUST_ACCEL_WITHIN_G * G:
            measured_up = accel / a
            estimated_up = self.R.T @ np.array([0, 0, 1.0])
            omega += FILTER_KP * np.cross(measured_up, estimated_up)
        self.R = self.R @ rotation_from_vector(omega * dt)
        # Re-orthonormalize so rounding doesn't build up
        u, _, vt = np.linalg.svd(self.R)
        self.R = u @ vt

    def roll_pitch(self):
        """Roll (+ = left side up) and pitch (+ = nose down), in radians."""
        return math.atan2(self.R[2, 1], self.R[2, 2]), -math.asin(np.clip(self.R[2, 0], -1, 1))


class FrontLegs:
    """Both front legs and their controller.

    Phases (both legs together):
      stance   hold the stance angle, leveled by the IMU loop
      push     straighten to push the nose up at takeoff
      tuck     fold up so the feet clear the floor
      reach    straighten a little and hold it until the accelerometer feels
               touchdown (free fall, then a spike), then back to stance
    """

    def __init__(self, design, model, data, rng=None, servo_left=None, servo_right=None):
        rng = rng if rng is not None else np.random.default_rng(0)
        self.data = data
        self.servos = {
            "left": servo_left or servo_from_design(design),
            "right": servo_right or servo_from_design(design),
        }
        for servo in self.servos.values():
            servo.linkage_ratio = 1.0  # front legs are direct drive

        def joint(name):
            return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

        self.qpos = {s: model.jnt_qposadr[joint(f"front_hip_{s}")] for s in SIDES}
        self.qvel = {s: model.jnt_dofadr[joint(f"front_hip_{s}")] for s in SIDES}
        self.actuator = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"front_servo_{s}")
                         for s in SIDES}

        self.leg_length = design.front_thigh_length + design.front_shin_length
        self.stance = front_stance_angle(design)
        self.full_push = math.radians(design.front_full_push_angle_deg)
        self.hip_range = tuple(math.radians(a) for a in design.front_hip_range_deg)
        self.tuck_angle = math.radians(TUCK_ANGLE_DEG)
        self.reach_angle = self.stance - math.radians(REACH_DEG)

        self.imu = Imu(model, data, rng)
        self.filter = None  # starts once the accelerometer reads about 1 g
        self.loop_period = 1.0 / CONTROL_RATE_HZ
        self.loop_timer = 0.0
        self.phase = "stance"
        self.phase_time = 0.0
        self.time_to_push = None
        self.felt_free_fall = False
        self.pending_targets = {s: self.stance for s in SIDES}  # sent this loop, arrive next loop
        self.targets = dict(self.pending_targets)  # what each servo is currently holding
        self.current = 0.0  # A, highest front-servo current on the last step

    # --- commands from the hop sequence ---------------------------------

    def fire(self, delay=FRONT_PUSH_DELAY):
        """Called when the first rear leg fires. Pushes after `delay` s."""
        self.time_to_push = delay

    # --- the controller loop ----------------------------------------------

    def hip_angle_for_length(self, length):
        """Hip angle that gives this hip-to-foot length (the knee keeps the foot below the hip)."""
        return math.acos(np.clip(length / self.leg_length, 0.0, 1.0))

    def leveled_targets(self, base_angle, roll, pitch, roll_rate, pitch_rate):
        base_length = self.leg_length * math.cos(base_angle)
        pitch_change = PITCH_KP * pitch + PITCH_KD * pitch_rate
        roll_change = ROLL_KP * roll + ROLL_KD * roll_rate
        targets = {}
        for side, sign in (("left", -1), ("right", +1)):
            change = np.clip(pitch_change + sign * roll_change, -MAX_LENGTH_CHANGE, MAX_LENGTH_CHANGE)
            targets[side] = self.hip_angle_for_length(base_length + change)
        return targets

    def control_loop(self, encoder_angles):
        gyro, accel = self.imu.read()
        near_1g = abs(np.linalg.norm(accel) - G) < TRUST_ACCEL_WITHIN_G * G
        if self.filter is None and near_1g:
            self.filter = AttitudeFilter(accel)  # start once it's sitting on the floor
        if self.filter is None:
            roll = pitch = roll_rate = pitch_rate = 0.0  # no attitude yet: don't level
        else:
            settled = self.phase == "stance" and self.phase_time > TRUST_ACCEL_AFTER_LANDING
            self.filter.update(gyro, accel, self.loop_period, trust_accel=settled)
            roll, pitch = self.filter.roll_pitch()
            roll_rate, pitch_rate = gyro[0], gyro[1]

        self.phase_time += self.loop_period
        if self.time_to_push is not None:
            self.time_to_push -= self.loop_period
            if self.time_to_push <= 0:
                self.time_to_push = None
                self.set_phase("push")

        if self.phase == "push":
            push_to = self.stance - FRONT_PUSH * (self.stance - self.full_push)
            targets = {s: push_to for s in SIDES}
            straight = all(abs(encoder_angles[s] - push_to) < math.radians(3) for s in SIDES)
            if straight or self.phase_time > MAX_FRONT_PUSH_TIME:
                self.set_phase("tuck")
        elif self.phase == "tuck":
            targets = {s: self.tuck_angle for s in SIDES}
            if self.phase_time > TUCK_TIME:
                self.set_phase("reach")
        elif self.phase == "reach":
            # Leveling in the air would land it on one stretched leg, so hold still until touchdown
            targets = {s: self.reach_angle for s in SIDES}
            g_felt = np.linalg.norm(accel) / G
            self.felt_free_fall |= g_felt < FREE_FALL_BELOW_G
            if (self.felt_free_fall and g_felt > TOUCHDOWN_ABOVE_G) or self.phase_time > MAX_REACH_TIME:
                self.set_phase("stance")
        else:
            targets = self.leveled_targets(self.stance, roll, pitch, roll_rate, pitch_rate)

        # Bus-servo latency: what we send now is what the servos hold next loop
        self.targets = self.pending_targets
        self.pending_targets = {s: float(np.clip(t, *self.hip_range)) for s, t in targets.items()}

    def set_phase(self, phase):
        self.phase = phase
        self.phase_time = 0.0
        self.felt_free_fall = False

    def step(self, dt):
        """Run the controller if a loop is due, then apply each servo's torque."""
        angles = {s: self.data.qpos[self.qpos[s]] for s in SIDES}
        speeds = {s: self.data.qvel[self.qvel[s]] for s in SIDES}

        self.loop_timer -= dt
        if self.loop_timer <= 0:
            self.loop_timer += self.loop_period
            self.control_loop({s: round(a / ENCODER_STEP) * ENCODER_STEP for s, a in angles.items()})

        # Inside each servo: its own position loop, limited by the torque-speed line
        self.current = 0.0
        for side in SIDES:
            servo = self.servos[side]
            self.data.ctrl[self.actuator[side]] = servo.hip_torque(self.targets[side], angles[side], speeds[side])
            self.current = max(self.current, servo.last_current)
