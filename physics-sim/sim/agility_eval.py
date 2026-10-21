"""Agility: how fast the bunny covers ground, turns, and follows a slalom.

Tests (each on 12 randomized robots the maps never saw, release servos on the PCA9685):
  straight   bound 10 hops, full push, straight: speed, time per hop, falls
  turn 90    turn to face 90° left, then stop: time until the true heading is within 15°, falls
  turn 180   same, all the way round
  slalom     bound 15 hops, the heading order flipping between +30° and -30° every 3 hops:
             how far the true heading is from the order on average, speed, falls

Controllers compared (every test sends the same BoundCommands, with heading_deg):
  baseline   BaselineController: ignores the heading, every bound hop straight at the commanded push
  bunny      BunnyController: front lead, ramp, and a heading loop on every bound hop

Run:  .venv/bin/python sim/agility_eval.py               (~3 min)
      .venv/bin/python sim/agility_eval.py --spring 10
"""

import argparse
import math
from multiprocessing import Pool

import numpy as np

from build_model import build_named
from config import DEFAULT_DESIGN, model_path
from controller import BaselineController, BoundCommand
from level2_big_hops import variant_name  # importing registers the stiffer-spring variants
from sweep import make_uncertain_bunny, random_uncertainty

ROBOTS = range(20, 32)
DESIGN = DEFAULT_DESIGN
FALL_TILT = 60.0
TURN_DONE_DEG = 15.0
SLALOM = (30.0, -30.0)
SLALOM_EVERY = 3
SLALOM_HOPS = 15

CONTROLLERS = {"baseline": BaselineController, "bunny": None}  # None: BunnyController


def robot(design_name, seed, controller):
    b = make_uncertain_bunny(design_name, random_uncertainty(np.random.default_rng(seed)), sensor_seed=seed,
                             controller_class=CONTROLLERS[controller])
    b.reset_to_crouch()
    b.run(0.5)
    return b


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def heading_deg(b):
    return math.degrees(b.heading())


def straight(design_name, seed, controller):
    b = robot(design_name, seed, controller)
    start, t0 = b.torso_position(), b.data.time
    b.send(BoundCommand(hops=10, push=1.0))
    b.step()
    fell = False
    while b.pending_command is not None or b.status != "idle":
        b.step()
        if b.tilt_deg() > FALL_TILT or b.data.time - t0 > 30:
            fell = True
            break
    t = b.data.time - t0
    moved = np.linalg.norm((b.torso_position() - start)[:2])
    return dict(fell=fell or b.tilt_deg() > 30, speed=moved / t * 100, per_hop=t / max(b.controller.hops_started, 1))


def turn(design_name, seed, controller, angle):
    b = robot(design_name, seed, controller)
    target = wrap(heading_deg(b) + angle)  # true target, for scoring
    est0 = math.degrees(b.estimated_yaw())
    t0, fell = b.data.time, False

    def done():
        return abs(wrap(heading_deg(b) - target)) < TURN_DONE_DEG

    b.send(BoundCommand(hops=40, push=1.0, heading_deg=wrap(est0 + angle), distance_cm=25))
    b.step()
    while not done():
        b.step()
        if b.tilt_deg() > FALL_TILT or b.data.time - t0 > 40 or (b.pending_command is None and b.status == "idle"):
            fell = b.tilt_deg() > FALL_TILT
            break
    return dict(fell=fell, time=b.data.time - t0 if done() else math.nan)


def slalom(design_name, seed, controller):
    b = robot(design_name, seed, controller)
    est0, true0 = math.degrees(b.estimated_yaw()), heading_deg(b)
    start, t0 = b.torso_position(), b.data.time
    errors, fell = [], False

    def order(hops):
        return SLALOM[(hops // SLALOM_EVERY) % 2]

    b.send(BoundCommand(hops=SLALOM_HOPS, push=1.0, heading_deg=wrap(est0 + order(0)), distance_cm=25))
    b.step()
    sent = 0
    while b.pending_command is not None or b.status != "idle":
        b.step()
        n = b.controller.hops_started
        if n // SLALOM_EVERY != sent and n < SLALOM_HOPS:  # the host flips the order, mid-bound
            sent = n // SLALOM_EVERY
            b.send(BoundCommand(hops=SLALOM_HOPS - n, push=1.0, heading_deg=wrap(est0 + order(n)), distance_cm=25))
        if n >= 1:
            errors.append(wrap(heading_deg(b) - wrap(true0 + order(max(n - 1, 0)))))
        if b.tilt_deg() > FALL_TILT or b.data.time - t0 > 60:
            fell = True
            break
    t = b.data.time - t0
    moved = np.linalg.norm((b.torso_position() - start)[:2])
    return dict(fell=fell or b.tilt_deg() > 30, rms=float(np.sqrt(np.mean(np.square(errors)))) if errors else math.nan,
                speed=moved / t * 100)


def job(args):
    test, design_name, seed, controller = args
    if test == "straight":
        return straight(design_name, seed, controller)
    if test == "turn 90":
        return turn(design_name, seed, controller, 90.0)
    if test == "turn 180":
        return turn(design_name, seed, controller, 180.0)
    return slalom(design_name, seed, controller)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--spring", type=float, default=7.0)
    parser.add_argument("--controllers", nargs="+", default=list(CONTROLLERS), choices=list(CONTROLLERS))
    args = parser.parse_args()
    name = DESIGN if args.spring == 7.0 else variant_name(DESIGN, args.spring)
    built = not model_path(name).exists()
    if built:
        build_named(name)

    tests = ("straight", "turn 90", "turn 180", "slalom")
    with Pool() as pool:
        results = {(t, c): pool.map(job, [(t, name, s, c) for s in ROBOTS])
                   for t in tests for c in args.controllers}

    print(f"{name}, {len(ROBOTS)} robots each\n")
    print(f"{'straight bound, 10 hops':26s} {'fell':>6s} {'speed':>10s} {'s/hop':>7s}")
    for c in args.controllers:
        r = results[("straight", c)]
        ok = [x for x in r if not x["fell"]] or r
        print(f"  {c:24s} {sum(x['fell'] for x in r):3d}/{len(r):<2d} {np.mean([x['speed'] for x in ok]):6.1f}cm/s"
              f" {np.mean([x['per_hop'] for x in ok]):6.2f}")
    for t in ("turn 90", "turn 180"):
        print(f"\n{t + ' (then stop)':26s} {'fell':>6s} {'time':>10s} {'turn rate':>10s} {'gave up':>8s}")
        for c in args.controllers:
            r = results[(t, c)]
            times = [x["time"] for x in r if not math.isnan(x["time"])]
            angle = float(t.split()[1])
            print(f"  {c:24s} {sum(x['fell'] for x in r):3d}/{len(r):<2d} {np.mean(times) if times else math.nan:8.1f} s"
                  f" {angle / np.mean(times) if times else math.nan:7.0f}°/s {len(r) - len(times) - sum(x['fell'] for x in r):6d}")
    print(f"\n{'slalom ±30°, flip every 3':26s} {'fell':>6s} {'heading err':>12s} {'speed':>10s}")
    for c in args.controllers:
        r = results[("slalom", c)]
        ok = [x for x in r if not x["fell"]] or r
        print(f"  {c:24s} {sum(x['fell'] for x in r):3d}/{len(r):<2d} {np.nanmean([x['rms'] for x in ok]):9.1f}° rms"
              f" {np.mean([x['speed'] for x in ok]):6.1f}cm/s")
    if built:
        model_path(name).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
