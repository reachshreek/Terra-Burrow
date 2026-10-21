"""Run the firmware (firmware/main.py) on a simulated robot, the way it would run on the real one.

Three things run here, each only through its own side of the Hal (sim_hal.py):

  the robot      physics + part models; the firmware's main loop drives it
  the host       sends the firmware text commands over the serial link, one at a
                 time, waiting for it to report idle (like a laptop script would)
  the mocap      scores each command from the sim's true state (the robot never sees this)

Commands (--do, repeatable; same words as the serial protocol):
  "hop 1 1 0"     push_left push_right right_delay_s
  "bound 10 1"    hops push
  "steer 90 12 25"  bound toward heading_deg [hops] [distance_cm per hop]

Run:  .venv/bin/python sim/run_firmware.py
      .venv/bin/python sim/run_firmware.py --do "hop 1 1 0" --do "steer 45 8" --seed 3
      .venv/bin/python sim/run_firmware.py --spring 10 --pico-timers --do "bound 10 1"
      .venv/bin/mjpython sim/run_firmware.py --view          (3D viewer; macOS needs mjpython)
"""

import argparse
import dataclasses
import math
import time

import numpy as np

from build_model import build_named
from config import DEFAULT_DESIGN, DESIGNS, model_path
from level2_big_hops import variant_name  # importing registers the stiffer-spring variants
from sim_hal import SimDone, SimHal
from sweep import make_uncertain_bunny, random_uncertainty

import firmware.main
import firmware.robot_config

DEFAULT_COMMANDS = ["hop 0.6 0.6 0", "hop 1 1 0", "bound 6 1", "steer 90 8"]


class Host:
    """The computer on the other end of the USB cable."""

    def __init__(self, hal, commands, quiet):
        self.hal, self.commands, self.quiet = hal, list(commands), quiet
        self.ready = False
        self.busy = False  # a command is out and the robot hasn't come back to idle
        self.seen_busy = False
        self.current = None
        self.on_command = []  # fn(command), when one is sent
        self.on_done = []  # fn(command), when the robot reports idle after it
        hal.link.on_line = self.on_line

    def on_line(self, t, line):
        if not self.quiet:
            print(f"  {t:7.3f}s  robot> {line}")
        words = line.split()
        if words[0] == "READY":
            self.ready = True
        elif words[0] == "ERR":
            self.busy = False  # the robot didn't understand it: move on
        elif words[0] == "STATUS" and self.busy:
            if words[1] != "idle":
                self.seen_busy = True
            elif self.seen_busy:
                self.busy = False
                for fn in self.on_done:
                    fn(self.current)

    def tick(self, hal):
        if not self.ready or self.busy:
            return
        if not self.commands:
            hal.stop()
            return
        self.current = self.commands.pop(0)
        self.busy, self.seen_busy = True, False
        if not self.quiet:
            print(f"  {hal.data.time:7.3f}s  host > {self.current.upper()}")
        hal.link.send_to_robot(self.current.upper())
        for fn in self.on_command:
            fn(self.current)


class Mocap:
    """Scores each command from the true state, in the frame the robot faced when it was sent."""

    def __init__(self, bunny):
        self.bunny = bunny
        self.start = None
        self.max_tilt = 0.0
        self.rows = []

    def command_sent(self, command):
        self.start = (self.bunny.torso_position(), self.bunny.heading())
        self.max_tilt = 0.0

    def step(self, hal):
        self.max_tilt = max(self.max_tilt, self.bunny.tilt_deg())

    def command_done(self, command):
        (p0, h0), b = self.start, self.bunny
        moved = b.torso_position() - p0
        fwd = math.cos(h0) * moved[0] + math.sin(h0) * moved[1]
        side = -math.sin(h0) * moved[0] + math.cos(h0) * moved[1]
        turned = b.heading() - h0
        self.rows.append(dict(command=command, distance=math.hypot(fwd, side) * 100,
                              direction=math.degrees(math.atan2(side, fwd)),
                              turn=math.degrees(math.atan2(math.sin(turned), math.cos(turned))),
                              max_tilt=self.max_tilt, upright=b.tilt_deg() < 30))

    def report(self):
        print(f"\nMocap (true state, the robot never sees it):")
        print(f"  {'command':16s} {'moved':>8s} {'direction':>10s} {'body turn':>10s} {'max tilt':>9s}  upright")
        for r in self.rows:
            print(f"  {r['command']:16s} {r['distance']:6.1f}cm {r['direction']:+9.1f}° {r['turn']:+9.1f}° "
                  f"{r['max_tilt']:8.1f}°  {'yes' if r['upright'] else 'NO'}")


class Viewer:
    def __init__(self, bunny):
        import mujoco.viewer
        self.viewer = mujoco.viewer.launch_passive(bunny.model, bunny.data)
        self.wall_start = time.time()
        self.steps = 0

    def step(self, hal):
        self.steps += 1
        if self.steps % 33:
            return
        if not self.viewer.is_running():
            hal.stop()
            return
        self.viewer.sync()
        ahead = hal.data.time - (time.time() - self.wall_start)
        if ahead > 0:
            time.sleep(ahead)


def robot_design(design, spring, pico_timers, built):
    name = design
    if spring != DESIGNS[design].latch_spring_stiffness:
        name = variant_name(design, spring)
        if name not in DESIGNS:
            DESIGNS[name] = dataclasses.replace(DESIGNS[design], latch_spring_stiffness=spring)
    if pico_timers:
        DESIGNS[f"{name}_pico"] = dataclasses.replace(DESIGNS[name], release_pwm_hz=None)
        name = f"{name}_pico"
    if not model_path(name).exists():
        build_named(name)
        built.append(name)
    return name


def main():
    parser = argparse.ArgumentParser(description="Run firmware/main.py on a simulated robot.")
    parser.add_argument("--design", default=DEFAULT_DESIGN, choices=DESIGNS)
    parser.add_argument("--no-controller", action="store_true", help="BaselineController: no control laws")
    parser.add_argument("--spring", type=float, default=7.0, help="latch spring, N·m/rad at the hip")
    parser.add_argument("--pico-timers", action="store_true", help="release servos on Pico timer pins, not the PCA9685")
    parser.add_argument("--seed", type=int, default=20, help="which randomized robot (0-9 built the hop maps)")
    parser.add_argument("--do", action="append", metavar="COMMAND", help=f"default: {DEFAULT_COMMANDS}")
    parser.add_argument("--max-seconds", type=float, default=120.0)
    parser.add_argument("--view", action="store_true", help="3D viewer, in real time")
    parser.add_argument("--quiet", action="store_true", help="don't print the serial traffic")
    args = parser.parse_args()

    built = []
    name = robot_design(args.design, args.spring, args.pico_timers, built)
    bunny = make_uncertain_bunny(name, random_uncertainty(np.random.default_rng(args.seed)), sensor_seed=args.seed)
    bunny.reset_to_crouch()

    hal = SimHal(bunny, seed=args.seed, duration=args.max_seconds)
    host, mocap = Host(hal, args.do or DEFAULT_COMMANDS, args.quiet), Mocap(bunny)
    host.on_command.append(mocap.command_sent)
    host.on_done.append(mocap.command_done)
    hal.on_step += [host.tick, mocap.step]
    if args.view:
        hal.on_step.append(Viewer(bunny).step)

    firmware.robot_config.DESIGN = name  # "flash" the firmware built for this robot
    firmware.robot_config.CONTROL_LAWS = not args.no_controller
    print(f"Robot: {name}, seed {args.seed}")
    try:
        firmware.main.main(hal)
    except SimDone:
        pass
    if host.commands or host.busy:
        print(f"\nStopped at {hal.data.time:.1f}s with commands left: {([host.current] if host.busy else []) + host.commands}")
    mocap.report()
    for n in built:
        model_path(n).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
