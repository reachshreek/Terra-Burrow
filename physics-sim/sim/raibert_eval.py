"""Score the Raibert-style loops (bunny_controller.py, law 3) on the front-swing design.

Bounds randomized robots (sweep.py's uncertainty + their own sensor seed) for 10 hops on
the current design and on latch_tethered_front_swing, at springs 10 and 13, with and
without height / speed targets. Then shows the loops tracking a target hop by hop.

  falls        runs that ended tipped past 30 deg
  ground       average forward speed over the run (cm/s)
  liftoff      forward speed at liftoff, true and as the robot measured it (cm/s)
  height       apex height from flight time, as the robot measured it (cm)

Run:  .venv/bin/python sim/raibert_eval.py              (~6 min)
      .venv/bin/python sim/raibert_eval.py --robots 12
"""

import argparse
import dataclasses
import math
from multiprocessing import Pool

import numpy as np

from build_model import build_named
from config import DESIGNS, model_path
from controller import BoundCommand
from sweep import make_uncertain_bunny, random_uncertainty

BASES = ("latch_tethered_front", "latch_tethered_front_swing")
SPRINGS = (10.0, 13.0)
CASES = [  # label, base design, BoundCommand fields
    ("current (no swing)", "latch_tethered_front", {}),
    ("Raibert, no targets", "latch_tethered_front_swing", {}),
    ("Raibert, height 10 cm", "latch_tethered_front_swing", {"height_cm": 10}),
    ("Raibert, speed 105 cm/s", "latch_tethered_front_swing", {"speed_cm_s": 105}),
    ("Raibert, height 10 + speed 95", "latch_tethered_front_swing", {"height_cm": 10, "speed_cm_s": 95}),
]
TRACKING = [("height 9 cm", {"height_cm": 9}), ("speed 100 cm/s", {"speed_cm_s": 100})]


def name(base, k):
    return f"{base}_k{k:g}"


for _base in BASES:  # registered on import, so worker processes know them too
    for _k in SPRINGS:
        DESIGNS[name(_base, _k)] = dataclasses.replace(DESIGNS[_base], latch_spring_stiffness=_k)


def run(job):
    design_name, seed, targets, hops = job
    b = make_uncertain_bunny(design_name, random_uncertainty(np.random.default_rng(seed)), sensor_seed=seed)
    b.reset_to_crouch()
    b.run(0.5)
    b.send(BoundCommand(hops=hops, **targets))
    b.step()
    t0, x0 = b.data.time, b.torso_position()
    true_liftoff = {}
    while b.pending_command is not None or b.status != "idle":
        b.step()
        r = b.controller.record
        if r is not None and not math.isnan(r.liftoff_time) and id(r) not in true_liftoff:
            h = b.heading()
            true_liftoff[id(r)] = math.cos(h) * b.data.qvel[0] + math.sin(h) * b.data.qvel[1]
        if b.tilt_deg() > 60 or b.data.time - t0 > 40:
            break
    hop_rows = [(r.liftoff_speed, true_liftoff[id(r)], r.hop_height) for r in b.controller.history
                if id(r) in true_liftoff and not math.isnan(r.flight_time)]
    ground = (b.torso_position()[0] - x0[0]) / (b.data.time - t0)
    return b.tilt_deg() > 30, ground, hop_rows


def main():
    parser = argparse.ArgumentParser(description="Score the Raibert-style loops.")
    parser.add_argument("--robots", type=int, default=36)
    args = parser.parse_args()
    seeds = range(20, 20 + args.robots)
    for base in BASES:
        for k in SPRINGS:
            build_named(name(base, k))

    with Pool() as pool:
        print(f"bounding 10 hops, {args.robots} robots          falls   ground  liftoff true / est   height")
        for k in SPRINGS:
            for label, base, targets in CASES:
                runs = pool.map(run, [(name(base, k), s, targets, 10) for s in seeds])
                hops = np.array([h for r in runs for h in r[2][1:]])
                print(f"  spring {k:g}  {label:30s} {sum(r[0] for r in runs):2d}/{len(runs)}   "
                      f"{np.mean([r[1] for r in runs]) * 100:5.1f}   {hops[:, 1].mean() * 100:5.1f} / {hops[:, 0].mean() * 100:5.1f}"
                      f"      {hops[:, 2].mean() * 100:4.1f}", flush=True)

        print("\ntracking, spring 10, front-swing design, mean over robots, hop by hop")
        for label, targets in TRACKING:
            runs = pool.map(run, [(name(BASES[1], 10.0), s, targets, 12) for s in seeds])
            col = 2 if "height_cm" in targets else 0
            table = np.full((len(runs), 12), np.nan)
            for i, r in enumerate(runs):
                for j, h in enumerate(r[2][:12]):
                    table[i, j] = h[col]
            print(f"  {label:16s} " + " ".join(f"{x * 100:5.1f}" for x in np.nanmean(table, 0)), flush=True)

    for base in BASES:
        for k in SPRINGS:
            model_path(name(base, k)).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
