"""Score a controller against the baseline: big hops, bounding.

Every test runs randomized robots (sweep.py's uncertainty + their own sensor
seed) the maps never saw, with the release servos on the PCA9685 (as in the
BOM) and on Pico timer pins (design.release_pwm_hz = None).

  big     3 calibration hops at half push, then 4 full-push hops, at springs 10 and 13.
          Share of full-push hops landing upright (a fall ends that robot's run)
  bound   10 bounding hops at springs 10 and 13. Runs that fall, hops done

Run:  .venv/bin/python sim/controller_eval.py             (~4 min)
      .venv/bin/python sim/controller_eval.py --only bound
"""

import argparse
import dataclasses
from multiprocessing import Pool

import numpy as np

import hop
from build_model import build_named
from config import DEFAULT_DESIGN, DESIGNS, model_path
from controller import BaselineController, BoundCommand, HopCommand
from bunny_controller import BunnyController
from level2_big_hops import variant_name  # importing registers the stiffer-spring variants
from sweep import make_uncertain_bunny, random_uncertainty

ROBOTS = range(20, 32)
CONTROLLERS = {"baseline": BaselineController, "bunny_controller": BunnyController}


def pico(name):
    return f"{name}_pico"


# Pico-timer twins of every design we test, registered on import so workers know them too
for _name in [variant_name(DEFAULT_DESIGN, 10), variant_name(DEFAULT_DESIGN, 13)]:
    DESIGNS[pico(_name)] = dataclasses.replace(DESIGNS[_name], release_pwm_hz=None)


def use_controller(name):
    hop.BunnyController = CONTROLLERS[name]


def robot(design_name, seed):
    b = make_uncertain_bunny(design_name, random_uncertainty(np.random.default_rng(seed)), sensor_seed=seed)
    b.reset_to_crouch()
    return b


def big_run(job):
    design_name, seed = job
    b = robot(design_name, seed)
    for _ in range(3):
        if not hop.simulate_hop(b, HopCommand(0.5, 0.5, 0)).upright:
            return []
    landed = []
    for _ in range(4):
        ok = hop.simulate_hop(b, HopCommand(1, 1, 0)).upright
        landed.append(ok)
        if not ok:
            break
    return landed


def bound_run(job):
    design_name, seed = job
    b = robot(design_name, seed)
    b.run(0.5)
    b.send(BoundCommand(hops=10, push=1.0))
    b.step()
    t0 = b.data.time
    while b.pending_command is not None or b.status != "idle":
        b.step()
        if b.tilt_deg() > 60 or b.data.time - t0 > 60:
            break
    return b.tilt_deg() > 30, b.controller.hops_started


def main():
    parser = argparse.ArgumentParser(description="Score bunny_controller.py against the baseline.")
    parser.add_argument("--only", choices=("big", "bound"))
    args = parser.parse_args()

    big_designs = [variant_name(DEFAULT_DESIGN, k) for k in (10, 13)]
    for name in big_designs:
        build_named(name)
    for name in list(DESIGNS):
        if name.endswith("_pico"):
            build_named(name)

    for controller in CONTROLLERS:
        print(f"\n=== {controller} ===")
        with Pool(initializer=use_controller, initargs=(controller,)) as pool:
            if args.only in (None, "big"):
                print("big hops (after 3 half-push hops):     full-push hops upright")
                for base in big_designs:
                    for name in (base, pico(base)):
                        runs = pool.map(big_run, [(name, s) for s in ROBOTS])
                        hops = [x for r in runs for x in r]
                        print(f"  {name:36s} {np.mean(hops) if hops else 0:4.0%}  ({sum(hops)}/{len(hops)}; "
                              f"{sum(1 for r in runs if not r)} fell in calibration)")
            if args.only in (None, "bound"):
                print("bounding 10 hops:                      runs that fell   hops done")
                for base in big_designs:
                    for name in (base, pico(base)):
                        runs = pool.map(bound_run, [(name, s) for s in ROBOTS])
                        print(f"  {name:36s} {sum(r[0] for r in runs):3d}/{len(runs)}       {np.mean([r[1] for r in runs]):4.1f}")

    for name in big_designs + [n for n in DESIGNS if n.endswith("_pico")]:
        model_path(name).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
