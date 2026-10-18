"""Watch the bunny hop around on its tether in the MuJoCo 3D viewer.

The bunny wanders forward using hops from its hop map (run sweep.py first). When
it gets within one turning circle of the end of the tether, it U-turns back
toward the anchor, the "tail tension" behavior.

Every hop moves the bunny forward, so it can't turn on the spot: its sharpest
reliable turn sets a turning radius, and it has to start turning back at least
that far before the cable runs out. Hitting the end of the cable mid-hop yanks
the tail and flips it.

Run (macOS needs mjpython for the viewer window):
    .venv/bin/mjpython sim/view.py --design latch_tethered --tether 2.5
Without a window (prints each hop instead):
    .venv/bin/python sim/view.py --design latch_tethered --no-window --hops 30
"""

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass

import mujoco
import mujoco.viewer
import numpy as np

from build_model import build_named
from config import DESIGNS, RESULTS_DIR
from hop import Bunny, HopCommand, simulate_hop
from sweep import USABLE_DIRECTION_SPREAD_DEG, USABLE_UPRIGHT_RATE, summarize


@dataclass
class HopMenu:
    """The hops the steering can choose from, picked out of the design's hop map."""
    straight: HopCommand
    turn_left: HopCommand
    turn_right: HopCommand
    wander: list  # gentle hops for wandering around
    turning_radius: float  # m
    longest_hop: float  # m


def load_hop_menu(design_name):
    path = RESULTS_DIR / f"hop_map_{design_name}.csv"
    if not path.exists():
        sys.exit(f"No hop map for '{design_name}'. Run: .venv/bin/python sim/sweep.py --design {design_name}")

    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        for key in ("push_left", "push_right", "distance_cm", "direction_deg", "heading_change_deg"):
            row[key] = float(row[key])
        row["right_delay_ms"] = int(row["right_delay_ms"])
        row["upright"] = row["upright"] == "True"

    usable = [s for s in summarize(rows)
              if s["upright_rate"] >= USABLE_UPRIGHT_RATE
              and s["direction_deg_spread"] <= USABLE_DIRECTION_SPREAD_DEG]

    def command(summary):
        push_left, push_right, delay_ms = summary["command"]
        return HopCommand(push_left, push_right, delay_ms / 1000.0)

    longest = max(s["distance_cm"] for s in usable)
    straight = max((s for s in usable if abs(s["direction_deg"]) < 3), key=lambda s: s["distance_cm"])
    # Sharpest turns that never fell in the sweep
    never_fell = [s for s in usable if s["upright_rate"] == 1.0]
    left = max(never_fell, key=lambda s: s["heading_change_deg"])
    right = min(never_fell, key=lambda s: s["heading_change_deg"])
    wander = [s for s in usable if abs(s["direction_deg"]) <= 12 and s["distance_cm"] >= 0.7 * longest]

    def radius(turn):
        return turn["distance_cm"] / 100 / math.radians(abs(turn["heading_change_deg"]))

    return HopMenu(
        straight=command(straight),
        turn_left=command(left),
        turn_right=command(right),
        wander=[command(s) for s in wander],
        turning_radius=max(radius(left), radius(right)),
        longest_hop=longest / 100,
    )


class TetherSteering:
    """Picks each hop: wander, or turn back toward the anchor before the tether runs out."""

    def __init__(self, menu: HopMenu, tether_length, seed=0):
        self.menu = menu
        self.tether_length = tether_length
        # Leave two full hops of slack beyond the turning circle: one because the
        # hop just before the check can carry it past this point, one as a margin
        self.turn_back_distance = tether_length - menu.turning_radius - 2 * menu.longest_hop
        self.rng = np.random.default_rng(seed)
        # Once a U-turn starts, keep turning the same way. Otherwise, with the
        # anchor straight behind, left and right turns alternate and cancel out.
        self.committed_turn = None

    def choose_hop(self, bunny):
        tail = bunny.data.site("tail").xpos
        anchor = bunny.data.site("tether_anchor").xpos
        distance_out = np.linalg.norm(tail - anchor)

        if distance_out > self.turn_back_distance:
            to_anchor = anchor - bunny.torso_position()
            bearing = math.atan2(to_anchor[1], to_anchor[0]) - bunny.heading()
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))  # wrap to ±180°
            if abs(bearing) > math.radians(20):
                if self.committed_turn is None:
                    self.committed_turn = self.menu.turn_left if bearing > 0 else self.menu.turn_right
                return self.committed_turn, "turning home"
            self.committed_turn = None
            return self.menu.straight, "heading home"

        self.committed_turn = None
        return self.menu.wander[self.rng.integers(len(self.menu.wander))], "wandering"

    def forget_turn(self):
        self.committed_turn = None


def main():
    parser = argparse.ArgumentParser(description="Watch the bunny hop on its tether.")
    parser.add_argument("--design", default="latch_tethered", choices=DESIGNS)
    parser.add_argument("--tether", type=float, default=2.5, help="tether length in m")
    parser.add_argument("--hops", type=int, default=1000)
    parser.add_argument("--no-window", action="store_true", help="print hops instead of opening the viewer")
    args = parser.parse_args()

    menu = load_hop_menu(args.design)
    steering = TetherSteering(menu, args.tether)
    print(f"Turning radius {menu.turning_radius:.2f} m, longest hop {menu.longest_hop * 100:.0f} cm: "
          f"turns back when the tail is {steering.turn_back_distance:.2f} m from the anchor")
    if steering.turn_back_distance < 0.3:
        sys.exit(f"A {args.tether} m tether is too short for this design to turn around. Try a longer one.")

    path = build_named(args.design, tether_length=args.tether)
    bunny = Bunny(args.design, model=mujoco.MjModel.from_xml_path(str(path)))
    bunny.reset_to_crouch()

    if args.no_window:
        for i in range(args.hops):
            command, reason = steering.choose_hop(bunny)
            result = simulate_hop(bunny, command)
            anchor = bunny.data.site("tether_anchor").xpos
            distance_from_anchor = np.linalg.norm(bunny.data.site("tail").xpos - anchor)
            print(f"hop {i + 1:3d} {reason:13s} moved {result.distance * 100:5.1f} cm, "
                  f"rose {result.apex_height * 100:4.1f} cm, turned {result.heading_change_deg:+5.1f}°, "
                  f"tail {distance_from_anchor:.2f} m from anchor, "
                  f"{'upright' if result.upright else 'FELL - resetting'}")
            if not result.upright:
                bunny.reset_to_crouch()
                steering.forget_turn()
        return

    with mujoco.viewer.launch_passive(bunny.model, bunny.data) as viewer:
        wall_start = time.time()
        sim_start = bunny.data.time

        def keep_real_time(b):
            # Redraw every ~1/60 s of sim time and wait so playback runs at real speed
            if int(b.data.time / b.model.opt.timestep) % 33 == 0:
                viewer.sync()
                ahead = (b.data.time - sim_start) - (time.time() - wall_start)
                if ahead > 0:
                    time.sleep(ahead)

        for _ in range(args.hops):
            if not viewer.is_running():
                break
            command, _ = steering.choose_hop(bunny)
            result = simulate_hop(bunny, command, on_step=keep_real_time)
            if not result.upright:
                print("Fell over - resetting to the start")
                bunny.reset_to_crouch()
                steering.forget_turn()


if __name__ == "__main__":
    main()
