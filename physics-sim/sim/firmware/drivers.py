"""Raw numbers to physics and back. One small class per part, all talking to the Hal.

  Imu           counts -> rad/s and m/s², with the boot-time gyro calibration
  Servos        encoder ticks <-> joint radians (through each servo's gearing); keeps the
                last good reading when a bus read times out
  Releases      trip/arm -> pulse widths, with sub-tick timing from a one-shot timer
  FootSwitches  active-low GPIO -> pressed, debounced
  HostLink      text commands from the host -> HopCommand / BoundCommand
"""

import math

import numpy as np

from controller import BoundCommand, HopCommand

from . import robot_config as cfg

G = 9.81


class Imu:
    def __init__(self, hal):
        self.hal = hal
        self.gyro_offset = np.zeros(3)  # counts

    def _counts(self):
        return np.array(self.hal.imu.read_raw(), dtype=float)

    def calibrate(self):
        """Average the gyro while sitting still; that average is its zero. Retries until it really was still."""
        while True:
            samples = []
            for _ in range(cfg.GYRO_CAL_SAMPLES):
                samples.append(self._counts()[3:])
                self.hal.clock.sleep_us(cfg.TICK_US)
            samples = np.array(samples)
            if np.all(samples.max(axis=0) - samples.min(axis=0) < cfg.GYRO_CAL_MAX_SPREAD):
                self.gyro_offset = samples.mean(axis=0)
                return self.gyro_offset / cfg.GYRO_COUNTS_PER_DPS  # °/s, for the boot log

    def read(self):
        c = self._counts()
        accel = (c[:3] - np.array(cfg.ACCEL_OFFSET)) / cfg.ACCEL_COUNTS_PER_G * G
        gyro = np.radians((c[3:] - self.gyro_offset) / cfg.GYRO_COUNTS_PER_DPS)
        return gyro, accel


class Servos:
    def __init__(self, hal):
        self.hal = hal
        self.names = cfg.servo_names()
        self.last = {n: None for n in self.names}  # joint rad, last good reading
        self.misses = 0  # bus reads that timed out, since boot

    def read(self):
        for name in self.names:
            sid, zero, direction = cfg.SERVOS[name]
            ticks = self.hal.servos.read_position(sid)
            if ticks is None:
                self.misses += 1
                continue
            servo_rad = direction * (ticks - zero) * cfg.RAD_PER_TICK
            self.last[name] = servo_rad / cfg.servo_turns_per_joint_turn(name)
        return dict(self.last)

    def write(self, commands):
        """commands: servo name -> ServoCommand (goal in joint rad)."""
        for name, command in commands.items():
            sid, zero, direction = cfg.SERVOS[name]
            servo_rad = command.goal * cfg.servo_turns_per_joint_turn(name)
            ticks = zero + direction * round(servo_rad / cfg.RAD_PER_TICK)
            self.hal.servos.write(sid, ticks, command.torque_on)


class Releases:
    def __init__(self, hal):
        self.hal = hal
        self.tripped = {}
        for side in cfg.RELEASE_CHANNEL:
            self._set(side, False)

    def _set(self, side, trip):
        self.tripped[side] = trip
        self.hal.pwm.set_pulse_us(cfg.RELEASE_CHANNEL[side], cfg.RELEASE_TRIP_US if trip else cfg.RELEASE_ARMED_US)

    def write(self, release, delay):
        """release: side -> trip? ; delay: side -> s after now to do it (finer than one tick)."""
        for side, trip in release.items():
            if trip == self.tripped[side]:
                continue
            d = delay.get(side, 0.0) if trip else 0.0
            if d > 0:
                self.tripped[side] = trip  # don't schedule it twice
                self.hal.clock.call_later_us(round(d * 1e6), lambda s=side, t=trip: self._set(s, t))
            else:
                self._set(side, trip)


class FootSwitches:
    def __init__(self, hal):
        self.hal = hal
        self.state = {s: False for s in cfg.FOOT_PIN}
        self.streak = {s: 0 for s in cfg.FOOT_PIN}

    def read(self):
        for side, pin in cfg.FOOT_PIN.items():
            pressed = self.hal.gpio.read(pin) == 0
            if pressed == self.state[side]:
                self.streak[side] = 0
            else:
                self.streak[side] += 1
                if self.streak[side] >= cfg.FOOT_DEBOUNCE_TICKS:
                    self.state[side], self.streak[side] = pressed, 0
        return dict(self.state)


class HostLink:
    """Line protocol from the host:
         HOP <push_left> <push_right> <right_delay_s>
         BOUND <hops> <push>
         STEER <heading_deg> [hops] [distance_cm]   bound toward a heading (the robot's own yaw, 0 = where
                                                    it faced at boot, + = left); sent mid-bound, it takes
                                                    over from the next hop without stopping
    """

    def __init__(self, hal):
        self.hal = hal

    def send(self, *words):
        self.hal.link.write(" ".join(str(w) for w in words))

    def poll(self):
        line = self.hal.link.readline()
        if line is None:
            return None
        words = line.split()
        try:
            kind, args = words[0].upper(), [float(w) for w in words[1:]]
            if kind == "HOP":
                return HopCommand(*args)
            if kind == "BOUND":
                return BoundCommand(int(args[0]), args[1])
            if kind == "STEER":
                hops = int(args[1]) if len(args) > 1 else 10
                distance = args[2] if len(args) > 2 else None
                return BoundCommand(hops, 1.0, heading_deg=args[0], distance_cm=distance)
        except (IndexError, ValueError, TypeError):
            pass
        self.send("ERR", repr(line))
        return None


def deg(rad):
    return f"{math.degrees(rad):+.1f}"
