"""Bound map: what each hop does when it's launched straight out of a landing.

The hop map (sweep.py) measures hops from standing still. Bounding hops start
from a body that's still moving, so they go differently. Here each randomized
robot bounds with a random command every hop (teeth and release delay), and
each hop is measured from its launch to the next launch: how far, which way,
and how much the body turned. bunny_controller.py fits its steering model
(controller/bound_model.py) from this.

Robots 0-9, so the evaluations on robots 20+ are on robots it never saw.

Run:  .venv/bin/python sim/bound_map.py              (~2 min)
      .venv/bin/python sim/bound_map.py --spring 10
"""

import argparse
import csv
import math
from multiprocessing import Pool

import numpy as np

import bunny_controller
from build_model import build_named
from config import DEFAULT_DESIGN, DESIGNS, RESULTS_DIR, model_path
from controller import BoundCommand, HopCommand
from controller.bound_model import MAX_ENERGY_DIFFERENCE, POSSIBLE_DELAYS, possible_pushes
from level2_big_hops import variant_name  # importing registers the stiffer-spring variants
from sweep import make_uncertain_bunny, random_uncertainty

ROBOTS = range(10)
RUNS_PER_ROBOT = 3
HOPS_PER_RUN = 25
WARMUP_HOPS = 2  # straight, not recorded
LOWEST_PUSH = 0.5


class RandomBounds(bunny_controller.BunnyController):
    """The real controller, except every bound hop is a random command."""

    rng = None

    def bound_hop(self):
        if self.bound_hops_done < WARMUP_HOPS:
            return HopCommand(0.7, 0.7, 0.0)
        while True:
            e_left, e_right = self.rng.choice(self.map_pushes, 2)
            if abs(e_left - e_right) <= MAX_ENERGY_DIFFERENCE:
                return HopCommand(e_left, e_right, float(self.rng.choice(POSSIBLE_DELAYS)))


def run(job):
    design_name, seed, run_index = job
    import hop
    hop.BunnyController = RandomBounds
    RandomBounds.rng = np.random.default_rng([seed, run_index])
    b = make_uncertain_bunny(design_name, random_uncertainty(np.random.default_rng(seed)), sensor_seed=seed)
    b.reset_to_crouch()
    b.controller.map_pushes = possible_pushes(b.design, LOWEST_PUSH)
    b.run(0.5)
    b.send(BoundCommand(hops=HOPS_PER_RUN, push=1.0))
    rows, launched, last = [], 0, None
    t0 = b.data.time
    while (b.pending_command is not None or b.status != "idle") and b.data.time - t0 < 4.0 * HOPS_PER_RUN:
        b.step()
        c = b.controller
        if c.hops_started != launched:  # a launch happened this tick: close the previous hop, open this one
            launched = c.hops_started
            pose = (b.torso_position(), b.heading())
            if last is not None and last["record"]:
                rows.append(measure(last, pose, b))
            legs = c.legs
            last = dict(pose=pose, record=launched > WARMUP_HOPS,
                        push_left=legs["left"].energy(), push_right=legs["right"].energy(),
                        right_delay_ms=round(c.plan.right_delay * 1000),
                        launch_state=launch_state(c))
        if b.tilt_deg() > 60:
            if last is not None and last["record"]:
                rows.append(dict(measure(last, (b.torso_position(), b.heading()), b), upright=False))
            break
    return rows


def launch_state(c):
    """What the robot itself knows at launch (IMU), to check whether it predicts the hop better."""
    roll, pitch = c.estimator.roll_pitch()
    g = c.estimator.gyro
    return dict(yaw_rate_dps=f"{math.degrees(g[2]):.1f}", roll_rate_dps=f"{math.degrees(g[0]):.1f}",
                pitch_rate_dps=f"{math.degrees(g[1]):.1f}", roll_deg=f"{math.degrees(roll):.1f}",
                pitch_deg=f"{math.degrees(pitch):.1f}")


def measure(last, pose, b):
    (p0, h0), (p1, h1) = last["pose"], pose
    moved = p1 - p0
    fwd = math.cos(h0) * moved[0] + math.sin(h0) * moved[1]
    side = -math.sin(h0) * moved[0] + math.cos(h0) * moved[1]
    turn = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
    return dict(push_left=f"{last['push_left']:.4f}", push_right=f"{last['push_right']:.4f}",
                right_delay_ms=last["right_delay_ms"], distance_cm=f"{math.hypot(fwd, side) * 100:.2f}",
                direction_deg=f"{math.degrees(math.atan2(side, fwd)):.2f}",
                heading_change_deg=f"{math.degrees(turn):.2f}", upright=b.tilt_deg() < 30, **last["launch_state"])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--design", default=DEFAULT_DESIGN, choices=DESIGNS)
    parser.add_argument("--spring", type=float, default=7.0)
    args = parser.parse_args()
    name = args.design if args.spring == 7.0 else variant_name(args.design, args.spring)
    built = not model_path(name).exists()
    if built:
        build_named(name)
    jobs = [(name, seed, k) for seed in ROBOTS for k in range(RUNS_PER_ROBOT)]
    with Pool() as pool:
        rows = [r for rs in pool.map(run, jobs) for r in rs]
    path = RESULTS_DIR / f"bound_map_{name}.csv"
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    falls = sum(not r["upright"] for r in rows)
    print(f"{len(rows)} bound hops on {len(ROBOTS)} robots, {falls} ended in a fall -> {path.relative_to(RESULTS_DIR.parent)}")
    if built:
        model_path(name).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
