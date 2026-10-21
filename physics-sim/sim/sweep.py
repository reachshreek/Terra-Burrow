"""Build the "hop map": which hop commands land upright, and where they go.

For every hop command in a grid (left push, right push, right-leg delay), run
several trials, each with the real-world uncertainties randomized: floor
friction, spring rates, servo strength, center-of-mass position, total mass
and servo timing jitter. Then:
  - save every trial to results/hop_map_<design>.csv
  - print a summary per command and the usable "cone": hop directions that
    land upright reliably AND go where they're aimed, trial after trial
  - plot landing spots and success rates to results/hop_map_<design>.png

Run:  python sim/sweep.py --design improved
      python sim/sweep.py --design bom --trials 20
"""

import argparse
import csv
import itertools
import math
from dataclasses import asdict, dataclass
from multiprocessing import Pool

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from build_model import build_named  # noqa: E402
from config import DESIGNS, RESULTS_DIR, model_path  # noqa: E402
from hop import Bunny, HopCommand, simulate_hop  # noqa: E402
from servo import servo_from_design  # noqa: E402

PUSH_LEVELS = (0.4, 0.7, 1.0)
DELAYS_MS = (-40, -30, -20, -10, 0, 10, 20, 30, 40)
# A command is "usable" if it lands upright this often...
USABLE_UPRIGHT_RATE = 0.95
# ...and its direction is repeatable (standard deviation across trials, degrees)
USABLE_DIRECTION_SPREAD_DEG = 10.0


# ---------------------------------------------------------------------------
# Randomizing the real-world uncertainties
# ---------------------------------------------------------------------------

@dataclass
class Uncertainty:
    floor_friction: float
    spring_scale_left: float
    spring_scale_right: float
    servo_strength_left: float
    servo_strength_right: float
    servo_speed_left: float
    servo_speed_right: float
    com_shift_mm: tuple
    mass_scale: float
    front_spring_scale: float
    timing_jitter_ms: float


def random_uncertainty(rng):
    return Uncertainty(
        floor_friction=rng.uniform(0.6, 1.2),  # smooth floor to grippy rug
        spring_scale_left=rng.uniform(0.9, 1.1),  # spring rate tolerance ±10%
        spring_scale_right=rng.uniform(0.9, 1.1),
        servo_strength_left=rng.uniform(0.9, 1.1),  # servo-to-servo variation
        servo_strength_right=rng.uniform(0.9, 1.1),
        servo_speed_left=rng.uniform(0.95, 1.05),
        servo_speed_right=rng.uniform(0.95, 1.05),
        com_shift_mm=tuple(rng.uniform(-5, 5, size=3)),  # mass estimates are rough
        mass_scale=rng.uniform(0.95, 1.05),
        front_spring_scale=rng.uniform(0.8, 1.2),
        timing_jitter_ms=rng.uniform(-5, 5),  # bus servo command latency
    )


def make_uncertain_bunny(design_name, u: Uncertainty, sensor_seed=0):
    model = mujoco.MjModel.from_xml_path(str(model_path(design_name)))
    design = DESIGNS[design_name]

    def joint(name):
        return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso")

    model.geom_friction[floor, 0] = u.floor_friction
    model.jnt_stiffness[joint("hip_left")] *= u.spring_scale_left
    model.jnt_stiffness[joint("hip_right")] *= u.spring_scale_right
    model.jnt_stiffness[joint("front_spring_left")] *= u.front_spring_scale
    model.jnt_stiffness[joint("front_spring_right")] *= u.front_spring_scale
    model.body_ipos[torso] += np.array(u.com_shift_mm) / 1000.0
    model.body_mass[torso] *= u.mass_scale
    model.body_inertia[torso] *= u.mass_scale

    return Bunny(
        design_name,
        model=model,
        servo_left=servo_from_design(design, u.servo_strength_left, u.servo_speed_left),
        servo_right=servo_from_design(design, u.servo_strength_right, u.servo_speed_right),
        sensor_seed=sensor_seed,  # IMU noise, bias and mounting error, for designs that use the IMU
    )


# ---------------------------------------------------------------------------
# Running the sweep
# ---------------------------------------------------------------------------

def run_trial(job):
    design_name, command, seed = job
    rng = np.random.default_rng(seed)
    u = random_uncertainty(rng)
    bunny = make_uncertain_bunny(design_name, u, sensor_seed=seed)
    bunny.reset_to_crouch()

    jittered = HopCommand(command.push_left, command.push_right,
                          command.right_delay + u.timing_jitter_ms / 1000.0)
    result = simulate_hop(bunny, jittered)

    return {
        "push_left": command.push_left,
        "push_right": command.push_right,
        "right_delay_ms": round(command.right_delay * 1000),
        "seed": seed,
        "distance_cm": result.distance * 100,
        "direction_deg": result.direction_deg,
        "heading_change_deg": result.heading_change_deg,
        "apex_rise_cm": result.apex_height * 100,
        "flight_ms": result.flight_time * 1000,
        "max_tilt_deg": result.max_tilt_deg,
        "upright": result.upright,
        "peak_current_a": result.peak_current,
        **{f"u_{k}": v for k, v in asdict(u).items()},
    }


def all_commands():
    for push_left, push_right, delay_ms in itertools.product(PUSH_LEVELS, PUSH_LEVELS, DELAYS_MS):
        yield HopCommand(push_left, push_right, delay_ms / 1000.0)


def command_key(row):
    return (row["push_left"], row["push_right"], row["right_delay_ms"])


def summarize(rows):
    """One summary per command: upright rate and average outcome of upright trials."""
    groups = {}
    for row in rows:
        groups.setdefault(command_key(row), []).append(row)

    summaries = []
    for key, trials in groups.items():
        upright = [t for t in trials if t["upright"]]
        summary = {"command": key, "upright_rate": len(upright) / len(trials)}
        for field in ("distance_cm", "direction_deg", "heading_change_deg"):
            values = [t[field] for t in upright]
            summary[field] = float(np.mean(values)) if values else math.nan
            summary[field + "_spread"] = float(np.std(values)) if values else math.nan
        summaries.append(summary)
    return summaries


def print_summary(design_name, summaries):
    print(f"\nHop map for design '{design_name}'")
    print(f"{'L push':>6} {'R push':>6} {'R delay':>8} {'upright':>8} {'distance':>12} "
          f"{'direction':>14} {'turn':>13}")
    for s in sorted(summaries, key=lambda s: s["command"]):
        push_left, push_right, delay_ms = s["command"]
        print(f"{push_left:6.1f} {push_right:6.1f} {delay_ms:+6d}ms {s['upright_rate']:7.0%} "
              f"{s['distance_cm']:6.1f}±{s['distance_cm_spread']:3.1f}cm "
              f"{s['direction_deg']:+6.1f}±{s['direction_deg_spread']:4.1f}° "
              f"{s['heading_change_deg']:+5.1f}±{s['heading_change_deg_spread']:4.1f}°")

    usable = [s for s in summaries
              if s["upright_rate"] >= USABLE_UPRIGHT_RATE
              and s["direction_deg_spread"] <= USABLE_DIRECTION_SPREAD_DEG]
    print(f"\n{len(usable)} of {len(summaries)} commands land upright at least "
          f"{USABLE_UPRIGHT_RATE:.0%} of the time with direction repeatable to "
          f"±{USABLE_DIRECTION_SPREAD_DEG:.0f}°.")
    if usable:
        directions = [s["direction_deg"] for s in usable]
        distances = [s["distance_cm"] for s in usable]
        turns = [s["heading_change_deg"] for s in usable]
        print(f"Usable cone: direction {min(directions):+.0f}° to {max(directions):+.0f}°, "
              f"distance {min(distances):.1f} to {max(distances):.1f} cm, "
              f"body turn {min(turns):+.0f}° to {max(turns):+.0f}° per hop")
    return usable


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
UPRIGHT_COLOR = "#2a78d6"  # categorical slot 1 (blue)
FELL_COLOR = "#e34948"  # categorical slot 8 (red), also drawn as an x


def plot(design_name, rows, summaries, usable, path):
    fig, (landing_ax, rate_ax) = plt.subplots(1, 2, figsize=(13, 5.5), facecolor=SURFACE)
    for ax in (landing_ax, rate_ax):
        ax.set_facecolor(SURFACE)
        ax.tick_params(colors=TEXT_SECONDARY, labelsize=9)
        for spine in ax.spines.values():
            spine.set_visible(False)

    # Left: where each trial landed, seen from above, relative to the start pose
    def landing_xy(row):
        angle = math.radians(row["direction_deg"])
        return row["distance_cm"] * math.cos(angle), row["distance_cm"] * math.sin(angle)

    upright = np.array([landing_xy(r) for r in rows if r["upright"]]).reshape(-1, 2)
    fell = np.array([landing_xy(r) for r in rows if not r["upright"]]).reshape(-1, 2)
    # Plot sideways (y) on the horizontal axis with left on the left, forward going up
    landing_ax.scatter(-upright[:, 1], upright[:, 0], s=14, color=UPRIGHT_COLOR, alpha=0.6,
                       edgecolors=SURFACE, linewidths=0.5, label=f"Landed upright ({len(upright)})")
    if len(fell):
        landing_ax.scatter(-fell[:, 1], fell[:, 0], s=28, color=FELL_COLOR, marker="x",
                           linewidths=1.5, label=f"Fell over ({len(fell)})")
    landing_ax.scatter([0], [0], s=60, color=TEXT_PRIMARY, marker="^", zorder=5, label="Start (facing up)")

    if usable:
        reach = max(s["distance_cm"] for s in usable) * 1.15
        for edge in (min(s["direction_deg"] for s in usable), max(s["direction_deg"] for s in usable)):
            a = math.radians(edge)
            landing_ax.plot([0, -reach * math.sin(a)], [0, reach * math.cos(a)],
                            color=TEXT_SECONDARY, linestyle="--", linewidth=1)
            side = "left" if edge > 0 else "right"
            landing_ax.annotate(f"{abs(edge):.0f}° {side}", (-reach * math.sin(a), reach * math.cos(a)),
                                color=TEXT_SECONDARY, fontsize=9, ha="center", va="bottom")

    # Zoom to the upright landings; falls that tumbled further away are counted in the legend
    half_width = max(np.abs(upright[:, 1]).max(), upright[:, 0].max() * 0.6) + 3
    landing_ax.set_xlim(-half_width, half_width)
    landing_ax.set_ylim(-3, upright[:, 0].max() + 5)
    landing_ax.set_aspect("equal")
    landing_ax.grid(color=GRID, linewidth=0.8)
    landing_ax.set_xlabel("sideways (cm, left is negative)", color=TEXT_SECONDARY, fontsize=9)
    landing_ax.set_ylabel("forward (cm)", color=TEXT_SECONDARY, fontsize=9)
    landing_ax.set_title("Where each hop landed", color=TEXT_PRIMARY, fontsize=11, loc="left")
    landing_ax.legend(frameon=False, fontsize=9, labelcolor=TEXT_PRIMARY,
                      loc="upper left", bbox_to_anchor=(1.0, 1.0))

    # Right: upright rate by timing offset and push asymmetry (averaged over overall push level)
    asymmetries = sorted({round(p_l - p_r, 1) for p_l, p_r in itertools.product(PUSH_LEVELS, PUSH_LEVELS)})
    grid = np.full((len(asymmetries), len(DELAYS_MS)), math.nan)
    for i, asym in enumerate(asymmetries):
        for j, delay in enumerate(DELAYS_MS):
            rates = [s["upright_rate"] for s in summaries
                     if round(s["command"][0] - s["command"][1], 1) == asym and s["command"][2] == delay]
            grid[i, j] = np.mean(rates)

    image = rate_ax.imshow(grid * 100, cmap="Blues", vmin=0, vmax=100, aspect="auto", origin="lower")
    rate_ax.set_xticks(range(len(DELAYS_MS)), [f"{d:+d}" for d in DELAYS_MS])
    rate_ax.set_yticks(range(len(asymmetries)), [f"{a:+.1f}" for a in asymmetries])
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            value = grid[i, j] * 100
            rate_ax.text(j, i, f"{value:.0f}", ha="center", va="center", fontsize=8,
                         color=SURFACE if value > 60 else TEXT_PRIMARY)
    rate_ax.set_xlabel("right leg delay (ms, + = left leg fires first)", color=TEXT_SECONDARY, fontsize=9)
    rate_ax.set_ylabel("left push minus right push", color=TEXT_SECONDARY, fontsize=9)
    rate_ax.set_title("% of trials landing upright", color=TEXT_PRIMARY, fontsize=11, loc="left")
    colorbar = fig.colorbar(image, ax=rate_ax, shrink=0.8)
    colorbar.outline.set_visible(False)
    colorbar.ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)

    fig.suptitle(f"Hop map: design '{design_name}'", color=TEXT_PRIMARY, fontsize=13, x=0.06, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor=SURFACE)
    plt.close(fig)


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Sweep hop commands with randomized uncertainty.")
    parser.add_argument("--design", default="improved", choices=DESIGNS)
    parser.add_argument("--trials", type=int, default=10, help="randomized trials per command")
    args = parser.parse_args()

    build_named(args.design)
    jobs = [(args.design, command, seed)
            for command in all_commands()
            for seed in range(args.trials)]
    print(f"Running {len(jobs)} hops ({len(jobs) // args.trials} commands x {args.trials} trials)...")
    with Pool() as pool:
        rows = pool.map(run_trial, jobs, chunksize=8)

    RESULTS_DIR.mkdir(exist_ok=True)
    csv_path = RESULTS_DIR / f"hop_map_{args.design}.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    summaries = summarize(rows)
    usable = print_summary(args.design, summaries)
    png_path = RESULTS_DIR / f"hop_map_{args.design}.png"
    plot(args.design, rows, summaries, usable, png_path)
    print(f"\nSaved {csv_path.relative_to(RESULTS_DIR.parent.parent)} and {png_path.name}")


if __name__ == "__main__":
    main()
