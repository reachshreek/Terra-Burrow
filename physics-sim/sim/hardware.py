"""The robot's physical parts, between the controller and the physics.

No decisions are made here. RobotHardware turns the controller's Outputs into
torques on the MuJoCo model, and the model's state into the controller's Inputs,
through models of the real parts:

  Imu           MPU-6050 or LSM6DS3 (HAL2-008, design.imu): bias, noise, mounting
                error, range limits
  BusServo      an ST3025 on the bus: commands arrive after a random delay, its
                own position loop runs on its own encoder (servo.py), and the
                host reads its position back at the bus rate. With
                design.servo_feedback False it's a PWM hobby servo (TD-8135MG):
                commands land at the next PWM frame and nothing reads back
  ReleaseServo  MG90D (HAL2-056) on the PCA9685 (HAL2-011): a new pulse only
                goes out at the next PWM frame, then the horn takes 14-22 ms
                (different per servo) to trip or re-arm the sear
  Ratchet       the pawl holds the leg on the highest tooth it has passed, but
                only against extension, and only while the sear is armed
  Foot switches rear-foot micro-switches (HAL2-009, Pololu 1404)
  Latch switches one per wind-and-latch leg (Pololu 1528): closed while the
                latch holds the leg fully wound

The true state is read here only to drive these models (an encoder sees the
true angle, a pawl catches on the true tooth, the IMU senses the true motion).
"""

import math

import mujoco
import numpy as np

from legs import latch_spring, tooth_pitch, winder_gear_ratio
from controller import Inputs, ServoCommand
from servo import Servo

SIDES = ("left", "right")
G = 9.81

# --- IMU at ±2000 °/s and ±16 g, after a gyro-bias calibration at power-up ---
# gyro/accel noise: rad/s and m/s² rms per sample. gyro bias: residual after calibration (random per
# robot). accel bias: zero-g offset (the accelerometer isn't calibrated).
IMUS = {
    "mpu6050": dict(gyro_noise=math.radians(0.07), gyro_bias=math.radians(0.5), accel_noise=0.055, accel_bias=0.2),
    # LSM6DS3 datasheet, high-performance mode at 416 Hz (~208 Hz bandwidth): 7 mdps/√Hz, 180 µg/√Hz at ±16 g,
    # zero-g offset ±40 mg. Residual gyro bias after the boot calibration kept as for the MPU-6050.
    "lsm6ds3": dict(gyro_noise=math.radians(0.007 * math.sqrt(208)), gyro_bias=math.radians(0.5),
                    accel_noise=180e-6 * G * math.sqrt(208), accel_bias=0.04 * G),
}
GYRO_RANGE = math.radians(2000)
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
    def __init__(self, model, data, rng, kind="mpu6050"):
        self.data = data
        self.rng = rng
        self.spec = IMUS[kind]
        self.gyro_addr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")]
        self.accel_addr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")]
        self.gyro_bias = rng.uniform(-self.spec["gyro_bias"], self.spec["gyro_bias"], 3)
        self.accel_bias = rng.uniform(-self.spec["accel_bias"], self.spec["accel_bias"], 3)
        self.mount = rotation_from_vector(rng.uniform(-MOUNT_ERROR, MOUNT_ERROR, 3))

    def read(self):
        gyro = self.mount @ self.data.sensordata[self.gyro_addr:self.gyro_addr + 3]
        accel = self.mount @ self.data.sensordata[self.accel_addr:self.accel_addr + 3]
        gyro = gyro + self.gyro_bias + self.rng.normal(0, self.spec["gyro_noise"], 3)
        accel = accel + self.accel_bias + self.rng.normal(0, self.spec["accel_noise"], 3)
        return np.clip(gyro, -GYRO_RANGE, GYRO_RANGE), np.clip(accel, -ACCEL_RANGE, ACCEL_RANGE)


class BusServo:
    """An ST3025 on the bus. Commands queue up and land after the bus delay.

    frame: a PwmClock for a PWM hobby servo instead: commands land at its next frame,
    and the servo reports nothing back."""

    def __init__(self, servo: Servo, start_goal, rng, frame=None):
        self.servo = servo
        self.rng = rng
        self.frame = frame
        self.command = ServoCommand(start_goal, True)
        self.queue = []  # (arrival time, ServoCommand)

    def send(self, command, now):
        arrival = self.frame.next_frame(now) if self.frame else now + self.rng.uniform(*BUS_COMMAND_DELAY)
        self.queue.append((arrival, command))

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
        return None if self.frame else self.servo.reported_hip_angle

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

    def __init__(self, crouch, pitch, start_angle):
        self.crouch = crouch
        self.pitch = pitch
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

    def fully_wound(self, armed, angle):
        """The latch switch: the pawl is on the top tooth (the crouch) and the leg is down on it."""
        return armed and self.caught is not None and self.caught >= self.crouch - 1e-6 and angle <= self.caught + LOCK_MARGIN


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
        self.imu = Imu(model, data, rng, design.imu)

        self.joints = {f"rear_{s}": joint(f"hip_{s}") for s in SIDES}
        self.actuators = {f"rear_{s}": nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"servo_{s}") for s in SIDES}
        crouch = math.radians(design.crouch_angle_deg)
        self.servos = {}
        for s in SIDES:
            # The winder: same servo, geared down to the hip
            spec = servo_specs[f"rear_{s}"].spec_copy(linkage_ratio=1.0 / winder_gear_ratio(design))
            self.servos[f"rear_{s}"] = BusServo(spec, crouch, rng)
        latch_front = design.front_drive == "latch"
        for s in SIDES:
            self.joints[f"front_{s}"] = joint(f"front_hip_{s}")
            self.actuators[f"front_{s}"] = nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"front_servo_{s}")
            # Latch front legs: a winder, geared like the rear ones. Servo front legs: direct drive.
            ratio = 1.0 / winder_gear_ratio(design, "front") if latch_front else 1.0
            self.servos[f"front_{s}"] = BusServo(servo_specs[f"front_{s}"].spec_copy(linkage_ratio=ratio),
                                                 front_stance, rng)
        self.qpos = {n: model.jnt_qposadr[j] for n, j in self.joints.items()}
        self.qvel = {n: model.jnt_dofadr[j] for n, j in self.joints.items()}

        pwm = PwmClock(design.release_pwm_hz or 50.0, rng)  # drawn either way, so a seed is the same robot
        if not design.release_pwm_hz:
            pwm = None
        self.release = {s: ReleaseServo(pwm, rng) for s in SIDES}
        self.peak_current = 0.0
        # Wind-and-latch legs: servo name -> its release key (Outputs.release), latch parts and spring
        self.latch = {f"rear_{s}": s for s in SIDES}
        self.ratchet, self.spring_actuator, self.pawl, self.spring = {}, {}, {}, {}

        def add_latch(name, key, leg, suffix):
            k, crouch, slack = latch_spring(design, leg)
            self.latch[name] = key
            self.ratchet[key] = Ratchet(crouch, tooth_pitch(design, leg), data.qpos[self.qpos[name]])
            self.spring_actuator[key] = nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"spring_{suffix}")
            self.pawl[key] = nid(mujoco.mjtObj.mjOBJ_EQUALITY, f"latch_{suffix}")
            self.spring[key] = (k, slack)

        for s in SIDES:
            add_latch(f"rear_{s}", s, "rear", s)

        if design.front_swing:  # made last, so a seed draws the same robot with or without them
            for s in SIDES:
                name = f"front_swing_{s}"
                self.joints[name] = joint(name)
                self.actuators[name] = nid(mujoco.mjtObj.mjOBJ_ACTUATOR, f"front_swing_servo_{s}")
                self.servos[name] = BusServo(servo_specs[name].spec_copy(linkage_ratio=1.0), 0.0, rng)
                self.qpos[name] = model.jnt_qposadr[self.joints[name]]
                self.qvel[name] = model.jnt_dofadr[self.joints[name]]

        # Made last too, so seeds draw the same robots as before for the servo-front designs
        if latch_front:
            for s in SIDES:
                self.release[f"front_{s}"] = ReleaseServo(pwm, rng)
                add_latch(f"front_{s}", f"front_{s}", "front", f"front_{s}")
        if not design.servo_feedback:  # PWM winders, on the PCA9685's 50 Hz frames
            frame = PwmClock(50.0, rng)
            for bus in self.servos.values():
                bus.frame = frame

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
            latch_switch={name: self.ratchet[key].fully_wound(not self.release[key].tripped,
                                                              self.data.qpos[self.qpos[name]])
                          for name, key in self.latch.items()},
        )

    def apply(self, outputs, now):
        for name, command in outputs.servo.items():
            self.servos[name].send(command, now)
        for key, trip in outputs.release.items():
            self.release[key].send(trip, now, outputs.release_delay.get(key, 0.0))

    # --- physics step ---

    def step(self, dt, now):
        current = 0.0
        for name, bus in self.servos.items():
            angle = self.data.qpos[self.qpos[name]]
            speed = self.data.qvel[self.qvel[name]]
            torque = bus.torque(angle, speed, dt, now)
            if name in self.latch:
                s = self.latch[name]
                if torque < 0:  # a cord can only pull the leg toward the crouch
                    torque, bus.servo.last_current = 0.0, 0.0
                k, slack = self.spring[s]
                spring = -k * max(0.0, angle - slack)
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
