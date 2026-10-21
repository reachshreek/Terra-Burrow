"""The robot's firmware: boot, then a 200 Hz loop of read -> control -> write.

This is the file you'd flash. It only talks to `hal` (see sim_hal.py for the
calls), so the same loop runs against the sim (run_firmware.py) or, with a
Pico 2 Hal behind it, the real robot.

Every tick:
  1. read the raw sensors and convert them      (drivers.py)
  2. pick up a command from the host, if any    (drivers.HostLink)
  3. run the controller                         (bunny_controller.py)
  4. convert its outputs and send them          (drivers.py)
  5. telemetry back to the host
  6. sleep until the next tick
"""

import math

from bunny_controller import BunnyController
from controller import BaselineController, Inputs

from . import robot_config as cfg
from .drivers import FootSwitches, HostLink, Imu, Releases, Servos, deg


def main(hal):
    host = HostLink(hal)
    imu, servos, releases, feet = Imu(hal), Servos(hal), Releases(hal), FootSwitches(hal)

    # --- Boot ---
    host.send("BOOT", cfg.DESIGN)
    hal.clock.sleep_us(200_000)  # let the body settle on its feet after power-up
    offset = imu.calibrate()
    host.send("GYRO_CAL", *(f"{o:+.2f}" for o in offset), "deg/s")
    controller = (BunnyController if cfg.CONTROL_LAWS else BaselineController)(cfg.design(), cfg.front_stance())
    host.send("READY")

    # --- Control loop ---
    status, hops_logged = None, 0
    next_tick = hal.clock.ticks_us()
    while True:
        gyro, accel = imu.read()
        inputs = Inputs(gyro=gyro, accel=accel, servo_position=servos.read(), foot_switch=feet.read(),
                        command=host.poll())

        out = controller.update(inputs)

        servos.write(out.servo)
        releases.write(out.release, out.release_delay)

        if out.status != status:
            status = out.status
            host.send("STATUS", status)
        while hops_logged < len(controller.history):  # what the robot measured about each hop it finished
            r = controller.history[hops_logged]
            hops_logged += 1
            host.send("HOP_DONE", hops_logged, "yaw", deg(r.yaw_change), "flight_ms",
                      "nan" if math.isnan(r.flight_time) else round(r.flight_time * 1000),
                      "peak_g", f"{r.peak_g:.1f}", "roll", deg(r.settled_roll), "pitch", deg(r.settled_pitch))

        next_tick += cfg.TICK_US
        hal.clock.sleep_until_us(next_tick)
