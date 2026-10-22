"""The robot's physical parts, between the controller and the physics.

No decisions are made here. RobotHardware turns the controller's Outputs into
torques on the MuJoCo model, and the model's state into the controller's Inputs,
through models of the real parts:

  Imu           MPU-6050 (HAL2-008): bias, noise, mounting error, range limits
  BusServo      an ST3025 on the bus: commands arrive after a random delay, its
                own position loop runs on its own encoder (servo.py), and the
                host reads its position back at the bus rate
  ReleaseServo  MG90D (HAL2-056) on the PCA9685 (HAL2-011): a new pulse only
                goes out at the next PWM frame, then the horn takes 14-22 ms
                (different per servo) to trip or re-arm the sear
  Ratchet       the pawl holds the leg on the highest tooth it has passed, but
                only against extension, and only while the sear is armed
  Foot switches rear-foot micro-switches (HAL2-009)

The true state is read here only to drive these models (an encoder sees the
true angle, a pawl catches on the true tooth, the IMU senses the true motion).
"""

import math

import mujoco
import numpy as np

from legs import winder_gear_ratio
from controller import Inputs, ServoCommand
from servo import Servo

SIDES = ("left", "right")
G = 9.81

# --- MPU-6050 at ±2000 °/s and ±16 g, after a gyro-bias calibration at power-up ---
GYRO_NOISE = math.radians(0.07)  # rad/s rms per sample
GYRO_BIAS = math.radians(0.5)  # rad/s, residual bias after calibration (random per robot)
GYRO_RANGE = math.radians(2000)
ACCEL_NOISE = 0.055  # m/s² rms per sample
ACCEL_BIAS = 0.2  # m/s², zero-g offset after calibration
ACCEL_RANGE = 16 * G
MOUNT_ERROR = math.radians(1.0)  # rad, how crooked the board might sit on the chassis

# --- Bus and release servos ---
BUS_COMMAND_DELAY = (0.0003, 0.0015)  # s, packet time plus the servo picking it up
RELEASE_TRAVEL_TIME = (0.014, 0.022)  # s, MG90D moving ~8° to trip the sear (range across servos)
RELEASE_JITTER = 0.001  # s rms, per move

LOCK_MARGIN = math.radians(0.5)  # the pawl engages once the leg is back down within this of its tooth


def rotation_from_vector(v):
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


class Imu:
    def __init__(self, model, data, rng):
        self.data = data
        self.rng = rng
        self.gyro_addr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")]
        self.accel_addr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")]
        self.gyro_bias = rng.uniform(-GYRO_BIAS, GYRO_BIAS, 3)
        self.accel_bias = rng.uniform(-ACCEL_BIAS, ACCEL_BIAS, 3)
        self.mount = rotation_from_vector(rng.uniform(-MOUNT_ERROR, MOUNT_ERROR, 3))

    def read(self):
        gyro = self.mount @ self.data.sensordata[self.gyro_addr:self.gyro_addr + 3]
        accel = self.mount @ self.data.sensordata[self.accel_addr:self.accel_addr + 3]
        gyro = gyro + self.gyro_bias + self.rng.normal(0, GYRO_NOISE, 3)
        accel = accel + self.accel_bias + self.rng.normal(0, ACCEL_NOISE, 3)
        return np.clip(gyro, -GYRO_RANGE, GYRO_RANGE), np.clip(accel, -ACCEL_RANGE, ACCEL_RANGE)


class BusServo:
    """An ST3025 on the bus. Commands queue up and land after the bus delay."""

    def __init__(self, servo: Servo, start_goal, rng):
        self.servo = servo
        self.rng = rng
        self.command = ServoCommand(start_goal, True)
        self.queue = []  # (arrival time, ServoCommand)

    def send(self, command, now):
        self.queue.append((now + self.rng.uniform(*BUS_COMMAND_DELAY), command))

    def torque(self, angle, speed, dt, now):
        while self.queue and self.queue[0][0] <= now:
            self.command = self.queue.pop(0)[1]
        torque = self.servo.hip_torque(self.command.goal, angle, speed, dt)
        if not self.command.torque_on:
            self.servo.last_current = 0.0
            return 0.0
        return torque

    @property
    def reported(self):
        return self.servo.reported_hip_angle

    @property
    def current(self):
        return self.servo.last_current


class PwmClock:
    """The PCA9685: a new pulse width reaches a servo at the start of the next frame, same moment for every channel."""

    def __init__(self, hz, rng):
        self.period = 1.0 / hz
        self.phase = rng.uniform(0, self.period)

    def next_frame(self, now):
        return self.phase + math.ceil((now - self.phase) / self.period) * self.period


class ReleaseServo:
    """pwm: the PCA9685's clock, or None when driven straight from a Pico timer pin."""

    def __init__(self, pwm, rng):
        self.pwm = pwm
        self.rng = rng
        self.travel = rng.uniform(*RELEASE_TRAVEL_TIME)  # this particular servo
        self.commanded = False
        self.tripped = False  # True = sear tripped, pawl lifted
        self.pending = None  # (time, value)

    def send(self, trip, now, delay=0.0):
        if trip != self.commanded:
            self.commanded = trip
            travel = max(0.0, self.travel + self.rng.normal(0, RELEASE_JITTER))
            start = now + delay
            if self.pwm is not None:
                start = self.pwm.next_frame(start)
            self.pending = (start + travel, trip)

    def step(self, now):
        if self.pending and now >= self.pending[0]:
            self.tripped = self.pending[1]
            self.pending = None


class Ratchet:
    """Teeth every `pitch` down from the crouch. Holds the highest tooth passed, against extension only."""

    def __init__(self, design, start_angle):
        self.crouch = math.radians(design.crouch_angle_deg)
        self.pitch = math.radians(design.ratchet_tooth_pitch_deg)
        self.caught = self.tooth_below(start_angle)
        self.was_armed = True

    def tooth_below(self, angle):
        teeth = math.ceil((self.crouch - angle) / self.pitch - 0.02)  # within 2% of a tooth counts as on it
        return self.crouch - max(teeth, 0) * self.pitch

    def locked(self, armed, angle, net_torque):
        if not armed:
            self.caught = None
        elif not self.was_armed or self.caught is None:
            self.caught = self.tooth_below(angle)  # the pawl drops onto the tooth below
        else:
            self.caught = max(self.caught, self.tooth_below(angle))  # clicks as the leg is wound up
        self.was_armed = armed
        # The pawl only carries load when the leg is pushed toward extension
        return armed and angle <= self.caught + LOCK_MARGIN and net_torque < 0


class RobotHardware:
    def __init__(self, design, model, data, rng, servo_specs, front_stance=None):
        """servo_specs: name -> Servo (its specs; fresh internal state is made here)."""
        self.design = design
        self.model = model
        self.data = data

        def nid(kind, name):
            return mujoco.mj_name2id(model, kind, name)

        def joint(name):
            return nid(mujoco.mjtObj.mjOBJ_JOINT, name)

        self.floor = nid(mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.rear_feet = {s: nid(mujoco.mjtObj.mjOBJ_GEOM, f"rear_foot_{s}") for s in SIDES}
        self.imu = Imu(model, data, rng)

        self.joints = {f"rear_{s}": joint(f"hip_{s}") for s in SIDES}
        self.actuators = {f"rear_{s}": nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"servo_{s}") for s in SIDES}
        crouch = math.radians(design.crouch_angle_deg)
        self.servos = {}
        for s in SIDES:
            # The winder: same servo, geared down to the hip
            spec = servo_specs[f"rear_{s}"].spec_copy(linkage_ratio=1.0 / winder_gear_ratio(design))
            self.servos[f"rear_{s}"] = BusServo(spec, crouch, rng)
        for s in SIDES:
            self.joints[f"front_{s}"] = joint(f"front_hip_{s}")
            self.actuators[f"front_{s}"] = nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"front_servo_{s}")
            self.servos[f"front_{s}"] = BusServo(servo_specs[f"front_{s}"].spec_copy(linkage_ratio=1.0),
                                                 front_stance, rng)
        self.qpos = {n: model.jnt_qposadr[j] for n, j in self.joints.items()}
        self.qvel = {n: model.jnt_dofadr[j] for n, j in self.joints.items()}

        pwm = PwmClock(design.release_pwm_hz or 50.0, rng)  # drawn either way, so a seed is the same robot
        if not design.release_pwm_hz:
            pwm = None
        self.release = {s: ReleaseServo(pwm, rng) for s in SIDES}
        self.ratchet = {s: Ratchet(design, data.qpos[self.qpos[f"rear_{s}"]]) for s in SIDES}
        self.spring_actuator = {s: nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"spring_{s}") for s in SIDES}
        self.pawl = {s: nid(mujoco.mjtObj.mjOBJ_EQUALITY, f"latch_{s}") for s in SIDES}
        self.spring_k = design.latch_spring_stiffness
        self.slack = math.radians(design.latch_spring_slack_angle_deg)
        self.peak_current = 0.0

        if design.front_swing:  # made last, so a seed draws the same robot with or without them
            for s in SIDES:
                name = f"front_swing_{s}"
                self.joints[name] = joint(name)
                self.actuators[name] = nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"front_swing_servo_{s}")
                self.servos[name] = BusServo(servo_specs[name].spec_copy(linkage_ratio=1.0), 0.0, rng)
                self.qpos[name] = model.jnt_qposadr[self.joints[name]]
                self.qvel[name] = model.jnt_dofadr[self.joints[name]]

    # --- controller interface ---

    def sample(self):
        gyro, accel = self.imu.read()
        touching = set()
        for c in self.data.contact[: self.data.ncon]:
            if c.geom1 == self.floor:
                touching.add(c.geom2)
            elif c.geom2 == self.floor:
                touching.add(c.geom1)
        return Inputs(
            gyro=gyro,
            accel=accel,
            servo_position={n: bus.reported for n, bus in self.servos.items()},
            foot_switch={s: self.rear_feet[s] in touching for s in SIDES},
        )

    def apply(self, outputs, now):
        for name, command in outputs.servo.items():
            self.servos[name].send(command, now)
        for s in SIDES:
            self.release[s].send(outputs.release[s], now, outputs.release_delay.get(s, 0.0))

    # --- physics step ---

    def step(self, dt, now):
        current = 0.0
        for name, bus in self.servos.items():
            angle = self.data.qpos[self.qpos[name]]
            speed = self.data.qvel[self.qvel[name]]
            torque = bus.torque(angle, speed, dt, now)
            if name.startswith("rear"):
                s = name[len("rear_"):]
                if torque < 0:  # a cord can only pull the leg toward the crouch
                    torque, bus.servo.last_current = 0.0, 0.0
                spring = -self.spring_k * max(0.0, angle - self.slack)
                self.release[s].step(now)
                armed = not self.release[s].tripped
                lock = self.ratchet[s].locked(armed, angle, torque + spring)
                self.data.ctrl[self.spring_actuator[s]] = spring
                if lock:
                    self.model.eq_data[self.pawl[s]][0] = self.ratchet[s].caught
                self.data.eq_active[self.pawl[s]] = lock
            self.data.ctrl[self.actuators[name]] = torque
            current = max(current, bus.current)
        self.peak_current = max(self.peak_current, current)
        return current
