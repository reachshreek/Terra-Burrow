"""Level 3: bounding. Hop again straight out of each landing, with no settling.

Compares two ways to cover ground with the same hops (full push, straight):
  one at a time   the normal HopCommand cycle: wind, sit still, hop, land, sit still
  bound           BoundCommand: relaunch as soon as the legs are wound and both rear feet are down

Each run is a randomized robot (uncertainty + its own sensor seed, as in
sweep.py) trying to do HOPS hops, with BaselineController (no control laws) and
with BunnyController. Reports ground speed, hop rate, and how many runs end in a fall.

Run:  python sim/level3_bounding.py              (~2 min)
      python sim/level3_bounding.py --spring 10  (a stiffer latch spring, as in level2_big_hops.py)
"""

import argparse
from multiprocessing import Pool

import numpy as np

from build_model import build_named
from config import DEFAULT_DESIGN, model_path
from controller import BaselineController
from hop import BoundCommand, HopCommand, simulate_hop
from level2_big_hops import SPRINGS, variant_name  # importing registers the stiffer-spring variants
from sweep import make_uncertain_bunny, random_uncertainty

HOPS = 10
CONTROLLERS = ("baseline", "bunny_controller")
MODES = ("one at a time", "bound")
FALL_TILT_DEG = 60.0  # true tilt beyond this at any point counts as a fall


def run(job):
    design_name, controller, mode, seed = job
    rng = np.random.default_rng(seed)
    bunny = make_uncertain_bunny(design_name, random_uncertainty(rng), sensor_seed=seed,
                                 controller_class=BaselineController if controller == "baseline" else None)
    bunny.reset_to_crouch()
    bunny.run(0.5)  # let it settle onto its feet after power-up
    start = bunny.torso_position()
    t0 = bunny.data.time
    fell = False

    if mode == "one at a time":
        hops = 0
        for _ in range(HOPS):
            result = simulate_hop(bunny, HopCommand(1, 1, 0))
            hops += 1
            if not result.upright:
                fell = True
                break
    else:
        bunny.send(BoundCommand(hops=HOPS, push=1.0))
        bunny.step()
        while bunny.pending_command is not None or bunny.status != "idle":
            bunny.step()
            if bunny.tilt_deg() > FALL_TILT_DEG:
                fell = True
                break
            if bunny.data.time - t0 > 6.0 * HOPS:
                break
        hops = bunny.controller.hops_started
        fell = fell or bunny.tilt_deg() > 30.0

    elapsed = bunny.data.time - t0
    moved = np.linalg.norm((bunny.torso_position() - start)[:2])
    return dict(controller=controller, mode=mode, fell=fell, hops=hops, elapsed=elapsed, moved=moved)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--runs", type=int, default=12, help="randomized robots per mode")
    parser.add_argument("--spring", type=float, default=7.0, choices=SPRINGS,
                        help="latch spring, N·m/rad at the hip (7 = current design)")
    args = parser.parse_args()
    name = DEFAULT_DESIGN if args.spring == 7.0 else variant_name(DEFAULT_DESIGN, args.spring)
    if args.spring != 7.0:
        build_named(name)
    jobs = [(name, c, m, seed) for c in CONTROLLERS for m in MODES for seed in range(args.runs)]
    print(f"Running {len(jobs)} runs of {HOPS} hops...")
    with Pool() as pool:
        rows = pool.map(run, jobs, chunksize=2)

    print(f"\n{name}\n{'controller':22s} {'mode':15s} {'fell':>6} {'hops/s':>7} {'speed':>9} {'per hop':>8}")
    for d in CONTROLLERS:
        for m in MODES:
            mine = [r for r in rows if r["controller"] == d and r["mode"] == m]
            ok = [r for r in mine if not r["fell"]] or mine
            rate = np.mean([r["hops"] / r["elapsed"] for r in ok])
            speed = np.mean([r["moved"] / r["elapsed"] for r in ok]) * 100
            per_hop = np.mean([r["moved"] / max(r["hops"], 1) for r in ok]) * 100
            fell = sum(r["fell"] for r in mine)
            print(f"{d:22s} {m:15s} {fell:3d}/{len(mine):<2d} {rate:6.2f}  {speed:5.1f}cm/s {per_hop:5.1f}cm")
    print(f"\nfell = runs that fell before finishing {HOPS} hops; rates and speeds average the runs that didn't. "
          "Speed is straight-line distance over time, so a robot that curves scores lower.")
    if args.spring != 7.0:
        model_path(name).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
