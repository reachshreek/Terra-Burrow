"""Level 2: bigger hops. How stiff a spring can the bunny land, with and without its control laws?

Turns up the latch-spring stiffness (more energy per hop) and runs each with
randomized uncertainty and a different robot per trial, like sweep.py, once
with BaselineController (no control laws) and once with BunnyController. For
every spring it reports how high and far the bunny goes and how often it lands
upright, for a straight hop and two gentle turns.

Run:  python sim/level2_big_hops.py            (~2 min)
      python sim/level2_big_hops.py --trials 20
"""

import argparse
import dataclasses
import math
from multiprocessing import Pool

import numpy as np

from build_model import build_named
from config import DEFAULT_DESIGN, DESIGNS, model_path
from controller import BaselineController
from hop import HopCommand, simulate_hop
from legs import winding_report
from sweep import make_uncertain_bunny, random_uncertainty

SPRINGS = (7.0, 10.0, 13.0, 16.0, 20.0)  # N·m/rad at the hip; 7 is the current design
BASES = (DEFAULT_DESIGN,)
CONTROLLERS = ("baseline", "bunny_controller")
COMMANDS = (HopCommand(1, 1, 0.0), HopCommand(1, 1, 0.01), HopCommand(1, 1, -0.01))


def variant_name(base, k):
    return f"{base}_k{k:g}"


# Register the variants on import, so worker processes know them too
for _base in BASES:
    for _k in SPRINGS:
        DESIGNS[variant_name(_base, _k)] = dataclasses.replace(DESIGNS[_base], latch_spring_stiffness=_k)


def run_trial(job):
    name, controller, command, seed = job
    rng = np.random.default_rng(seed)
    bunny = make_uncertain_bunny(name, random_uncertainty(rng), sensor_seed=seed,
                                 controller_class=BaselineController if controller == "baseline" else None)
    bunny.reset_to_crouch()
    r = simulate_hop(bunny, command)
    return dict(name=name, controller=controller, upright=r.upright, rise=r.apex_height, distance=r.distance,
                max_tilt=r.max_tilt_deg, direction=r.direction_deg)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=12, help="randomized robots per command")
    args = parser.parse_args()

    names = [variant_name(b, k) for b in BASES for k in SPRINGS]
    for name in names:
        build_named(name)
    jobs = [(name, ctl, c, seed) for name in names for ctl in CONTROLLERS for c in COMMANDS
            for seed in range(args.trials)]
    print(f"Running {len(jobs)} hops...")
    with Pool() as pool:
        rows = pool.map(run_trial, jobs, chunksize=4)

    print(f"\n{'spring':>8} {'energy':>7} {'pawl':>6} {'wind':>6} | "
          f"{'baseline: upright  rise   dist  tilt':>40} | {'bunny_controller: upright  rise   dist  tilt':>46}")
    for k in SPRINGS:
        w = winding_report(DESIGNS[variant_name(BASES[0], k)])
        line = f"{k:5.0f} N·m {w['energy_j']:5.1f} J {w['pawl_force_n']:4.0f} N {w['wind_time_s']:4.2f} s |"
        for ctl in CONTROLLERS:
            mine = [r for r in rows if r["name"] == variant_name(BASES[0], k) and r["controller"] == ctl]
            up = [r for r in mine if r["upright"]]
            rise = np.mean([r["rise"] for r in up]) * 100 if up else math.nan
            dist = np.mean([r["distance"] for r in up]) * 100 if up else math.nan
            tilt = np.mean([r["max_tilt"] for r in up]) if up else math.nan
            line += f"   {len(up) / len(mine):18.0%} {rise:4.0f}cm {dist:4.0f}cm {tilt:3.0f}° |"
        print(line)
    print("\nupright = share of all trials (straight + ±10 ms turns) landing upright; "
          "rise/dist/tilt = averages over the upright ones; energy and pawl force per leg")

    for name in names:  # the variant models were only needed for this run
        model_path(name).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
