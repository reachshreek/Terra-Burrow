"""The simulated robot as the firmware sees it: raw sensor values in, raw commands out.

This is the boundary between the sim and firmware/. The firmware only ever
talks to a Hal object, and on the real robot a Pico 2 version of the same
object would wrap the real drivers (I2C, the servo bus UART, PWM, GPIO, USB
serial). Everything here is in the units the chips themselves use:

  hal.imu.read_raw()                 MPU-6050 counts (ax, ay, az, gx, gy, gz), int16
                                     at ±16 g (2048 counts/g) and ±2000 °/s (16.4 counts per °/s).
                                     Has its factory gyro offset: calibrate at boot.
  hal.servos.read_position(id)       ST3025 encoder ticks 0-4095 at the servo shaft, or None
                                     when the bus read times out (it happens)
  hal.servos.write(id, ticks, torque_on)   goal position and torque enable, one bus packet
  hal.pwm.set_pulse_us(channel, us)  release-servo pulse width (PCA9685 or Pico timer pin)
  hal.gpio.read(pin)                 foot switches: 0 = pressed (pull-up, switch to ground), bounces
  hal.clock.ticks_us()               microseconds since power-up
  hal.clock.sleep_until_us(t)        block until then (here, that's when the physics runs)
  hal.clock.call_later_us(dt, fn)    one-shot timer, for sub-tick release timing
  hal.link.readline() / write(line)  text lines to and from the host computer (USB serial)

Time only moves inside sleep_until_us(). Reads and writes take no time, so the
firmware's main loop runs as if the Pico were infinitely fast; the bus
latencies and PWM frames are still modeled where the parts are (hardware.py).

ASSEMBLY below is how this particular robot is put together (servo IDs, which
way each horn faces, where its zero is). The firmware has its own copy in
firmware/robot_config.py, just as on the real robot you'd measure it once.
"""

import heapq
import math

import mujoco
import numpy as np

from controller import ServoCommand
from servo import ENCODER_STEP

SIDES = ("left", "right")
G = 9.81

ASSEMBLY = {
    # servo name: (bus ID, encoder ticks at joint angle 0, +1 or -1 if the horn faces the other way)
    "servos": {
        "rear_left": (1, 512, +1),
        "rear_right": (2, 3584, -1),
        "front_left": (3, 1024, +1),
        "front_right": (4, 3072, -1),
    },
    "release_channel": {"left": 0, "right": 1},  # PCA9685 channel (or Pico pin's slot)
    "release_armed_us": 1500,  # MG90D pulse that holds the sear...
    "release_trip_us": 1550,  # ...and the one that trips it (~8° of horn travel)
    "foot_pin": {"left": 14, "right": 15},  # GPIO, pulled up, switch to ground
}

# --- MPU-6050 raw output ---
ACCEL_COUNTS_PER_G = 2048.0  # AFS_SEL = 3, ±16 g
GYRO_COUNTS_PER_DPS = 16.4  # FS_SEL = 3, ±2000 °/s
GYRO_FACTORY_OFFSET = math.radians(5.0)  # rad/s, per axis, removed by the firmware's boot calibration
GYRO_WARMUP = 20.0  # s; the residual bias (hardware.Imu) creeps in over this as the chip warms up,
#                    so the boot calibration can't catch it

# --- Bus, foot switches, host link ---
BUS_READ_FAIL = 0.002  # chance a position read gets no reply in time
SWITCH_BOUNCE = (0.0005, 0.003)  # s of chatter after each make or break
LINK_LATENCY = (0.001, 0.004)  # s, USB serial, each way


class SimDone(Exception):
    """Raised inside sleep_until_us() when the run is over. The firmware doesn't need to catch it."""


class Clock:
    def __init__(self, hal):
        self.hal = hal
        self.timers = []  # heap of (time, seq, fn)
        self.seq = 0
        self.override = None  # the exact time, while a timer callback runs

    def now(self):
        return self.override if self.override is not None else self.hal.data.time

    def ticks_us(self):
        return int(round(self.now() * 1e6))

    def sleep_until_us(self, t_us):
        while self.hal.data.time < t_us * 1e-6 - 1e-9:
            self.hal.step()

    def sleep_us(self, dt_us):
        self.sleep_until_us(self.ticks_us() + dt_us)

    def call_later_us(self, delay_us, fn):
        self.seq += 1
        heapq.heappush(self.timers, (self.now() + max(0, delay_us) * 1e-6, self.seq, fn))

    def run_due(self):
        while self.timers and self.timers[0][0] <= self.hal.data.time + 1e-12:
            t, _, fn = heapq.heappop(self.timers)
            self.override = t
            try:
                fn()
            finally:
                self.override = None


class Imu:
    def __init__(self, hal, rng):
        self.hal = hal
        self.offset = rng.uniform(-GYRO_FACTORY_OFFSET, GYRO_FACTORY_OFFSET, 3)

    def read_raw(self):
        imu = self.hal.hardware.imu
        gyro, accel = imu.read()  # true motion + noise + residual bias + mounting error, range-limited
        warm = min(1.0, self.hal.data.time / GYRO_WARMUP)
        gyro = gyro - imu.gyro_bias * (1 - warm) + self.offset
        a = np.clip(np.round(accel / G * ACCEL_COUNTS_PER_G), -32768, 32767).astype(int)
        g = np.clip(np.round(np.degrees(gyro) * GYRO_COUNTS_PER_DPS), -32768, 32767).astype(int)
        return (*a.tolist(), *g.tolist())


class ServoBus:
    def __init__(self, hal, rng):
        self.hal = hal
        self.rng = rng
        hw = hal.hardware
        self.by_id = {}
        for name, (sid, zero, direction) in ASSEMBLY["servos"].items():
            if name in hw.servos:
                self.by_id[sid] = (name, zero, direction)

    def _ratio(self, name):
        return self.hal.hardware.servos[name].servo.linkage_ratio  # joint rad per servo rad

    def read_position(self, sid):
        if sid not in self.by_id or self.rng.random() < BUS_READ_FAIL:
            return None
        name, zero, direction = self.by_id[sid]
        hw = self.hal.hardware
        angle = self.hal.data.qpos[hw.qpos[name]]
        counts = round(angle / self._ratio(name) / ENCODER_STEP)
        return (zero + direction * counts) % 4096

    def write(self, sid, ticks, torque_on=True):
        if sid not in self.by_id:
            return
        name, zero, direction = self.by_id[sid]
        ticks = int(np.clip(ticks, 0, 4095))
        goal = direction * (ticks - zero) * ENCODER_STEP * self._ratio(name)
        self.hal.hardware.servos[name].send(ServoCommand(goal, bool(torque_on)), self.hal.clock.now())


class Pwm:
    def __init__(self, hal):
        self.hal = hal
        self.side = {ch: s for s, ch in ASSEMBLY["release_channel"].items()}
        self.threshold = (ASSEMBLY["release_armed_us"] + ASSEMBLY["release_trip_us"]) / 2

    def set_pulse_us(self, channel, us):
        hw = self.hal.hardware
        if channel not in self.side:
            return
        hw.release[self.side[channel]].send(us >= self.threshold, self.hal.clock.now())


class Gpio:
    def __init__(self, hal, rng):
        self.hal = hal
        self.rng = rng
        self.side = {pin: s for s, pin in ASSEMBLY["foot_pin"].items()}
        self.pressed = {s: False for s in SIDES}
        self.bounce_until = {s: -1.0 for s in SIDES}

    def update(self, touching):
        now = self.hal.data.time
        for s in SIDES:
            down = self.hal.hardware.rear_feet[s] in touching
            if down != self.pressed[s]:
                self.pressed[s] = down
                self.bounce_until[s] = now + self.rng.uniform(*SWITCH_BOUNCE)

    def read(self, pin):
        s = self.side.get(pin)
        if s is None:
            return 1
        if self.hal.data.time < self.bounce_until[s]:
            return int(self.rng.random() < 0.5)
        return 0 if self.pressed[s] else 1


class Link:
    """USB serial to the host. The host side is whatever the runner plugs into on_line / send_to_robot."""

    def __init__(self, hal, rng):
        self.hal = hal
        self.rng = rng
        self.to_robot = []  # (arrival time, line)
        self.to_host = []
        self.on_line = None  # host callback: fn(time, line)

    # firmware side
    def readline(self):
        if self.to_robot and self.to_robot[0][0] <= self.hal.data.time:
            return self.to_robot.pop(0)[1]
        return None

    def _arrival(self, queue, now):
        """A serial line keeps its order: nothing arrives before what was sent ahead of it."""
        t = now + self.rng.uniform(*LINK_LATENCY)
        return max(t, queue[-1][0]) if queue else t

    def write(self, line):
        self.to_host.append((self._arrival(self.to_host, self.hal.clock.now()), line))

    # host side
    def send_to_robot(self, line):
        self.to_robot.append((self._arrival(self.to_robot, self.hal.data.time), line))

    def deliver(self):
        while self.to_host and self.to_host[0][0] <= self.hal.data.time:
            t, line = self.to_host.pop(0)
            if self.on_line:
                self.on_line(t, line)


class SimHal:
    """A powered-up simulated robot. bunny: a hop.Bunny (its physics and hardware models; its own controller is unused)."""

    def __init__(self, bunny, seed=0, duration=None):
        self.bunny = bunny
        self.model, self.data, self.hardware = bunny.model, bunny.data, bunny.hardware
        rng = np.random.default_rng([seed, 7])  # separate stream, so the hardware models' draws match the Bunny's
        self.clock = Clock(self)
        self.imu = Imu(self, rng)
        self.servos = ServoBus(self, rng)
        self.pwm = Pwm(self)
        self.gpio = Gpio(self, rng)
        self.link = Link(self, rng)
        self.duration = duration
        self.stopped = False
        self.on_step = []  # fn(hal), after every physics step: the runner's host, mocap, viewer
        self.gpio.update(self.touching())

    def stop(self):
        self.stopped = True

    def touching(self):
        floor = self.hardware.floor
        out = set()
        for c in self.data.contact[: self.data.ncon]:
            if c.geom1 == floor:
                out.add(c.geom2)
            elif c.geom2 == floor:
                out.add(c.geom1)
        return out

    def step(self):
        if self.stopped or (self.duration is not None and self.data.time >= self.duration):
            raise SimDone
        self.clock.run_due()
        self.bunny.current = self.hardware.step(self.model.opt.timestep, self.data.time)
        mujoco.mj_step(self.model, self.data)
        self.gpio.update(self.touching())
        self.link.deliver()
        for fn in self.on_step:
            fn(self)
