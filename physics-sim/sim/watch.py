"""Watch the bunny: a live 3D window, or GIF clips you can share.

Modes (what the bunny is told to do, over and over, for --seconds):
  hop     one straight full hop, then sit (good in slow motion)
  steps   one hop at a time: wind, sit still, hop, land, repeat
  bound   bounding: relaunch straight out of each landing (BoundCommand)
  stand   no hops: just stand (use with --shove to test the front-leg leveling PD)

--shove N pushes the torso with N newtons, alternately forward and backward, for
SHOVE_TIME s every SHOVE_EVERY s, at the top of the shell so it pitches the body. With
--mode stand it tests the leveling PD on its own: compare with --no-controller, which has no
leveling (the front legs just hold their stance angle). --shove-side pushes sideways instead (roll).

Each run is a randomized robot like in the sweeps (--seed picks which one; the
same seed means the same robot and floor every time). The camera follows the
torso from its left side, swinging to keep up as it turns.

--live shows live graphs over the 3D view (pitch, height, heading, speed, and a stats
panel; see live_plots.py). --no-plots turns them off.

--camera picks the live view: "chase" (default) follows the bunny from its left side and
swings round as it turns, "follow" follows it from a fixed direction, "free" is the viewer's
own mouse-driven camera. In chase/follow you can still zoom and tilt with the mouse.

--no-controller runs BaselineController instead of BunnyController: the same
machinery (estimator, leg sequencing, front-leg leveling) without the control
laws in bunny_controller.py, for a before/after.

Run (macOS needs mjpython for --live):
    .venv/bin/mjpython sim/watch.py --spring 10 --mode bound --live
    .venv/bin/mjpython sim/watch.py --spring 10 --mode bound --live --no-controller   # the "before"
    .venv/bin/mjpython sim/watch.py --design latch_tethered_front_swing --spring 10 --live --height 12 --speed 70
                                                                     # Raibert loops (front shoulder swing)
    .venv/bin/python sim/watch.py --mode bound --seconds 6     # -> results/*.gif
    .venv/bin/python sim/watch.py --preset before_after --spring 10 --mode bound   # side by side GIF
    .venv/bin/python sim/watch.py --preset level2    # side by side: spring 7 vs 13 vs 13 without control laws
    .venv/bin/python sim/watch.py --preset level3    # side by side: one at a time vs bounding vs bounding at spring 10
"""

import argparse
import dataclasses
import math
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from build_model import build_named
from config import DEFAULT_DESIGN, DESIGNS, RESULTS_DIR, model_path
from controller import BaselineController
from hop import BoundCommand, HopCommand
from level2_big_hops import SPRINGS, variant_name  # importing registers the stiffer-spring variants
from sweep import make_uncertain_bunny, random_uncertainty

FPS = 20
WIDTH, HEIGHT = 400, 300
START_DELAY = 0.4  # s of standing before the first command
SHOVE_EVERY = 2.0  # s between shoves (--shove)
SHOVE_TIME = 0.5  # s each shove lasts
SHOVE_HEIGHT = 0.055  # m above the torso origin where the shove lands (top of the shell)
SHOVE = dict(newtons=0.0, side=False)  # set from --shove / --shove-side
RAIBERT = dict(height_cm=None, speed_cm_s=None)  # bound targets, set from --height / --speed


BUILT = set()  # stiffer-spring models built for this run, deleted at the end


def design_for(design, spring):
    if spring == 7.0:
        return design
    name = variant_name(design, spring)
    if name not in DESIGNS:  # e.g. a stiffer spring on the front-swing design
        DESIGNS[name] = dataclasses.replace(DESIGNS[design], latch_spring_stiffness=spring)
    if not model_path(name).exists():
        BUILT.add(name)  # ours to delete at the end
    build_named(name)  # always: a leftover file could predate edits to the builder or config
    return name


def make_bunny(design, spring, seed, controller=True):
    """controller False: BaselineController, with no control laws."""
    name = design_for(design, spring)
    bunny = make_uncertain_bunny(name, random_uncertainty(np.random.default_rng(seed)), sensor_seed=seed,
                                 controller_class=None if controller else BaselineController)
    bunny.reset_to_crouch()
    return bunny


def shove(bunny):
    """--shove: push the top of the shell forward, then backward (or left, then right), every SHOVE_EVERY s."""
    d, t = bunny.data, bunny.data.time
    d.xfrc_applied[bunny.torso] = 0.0
    if SHOVE["newtons"] <= 0 or t < START_DELAY + 1.0:
        return
    k, phase = divmod(t - START_DELAY - 1.0, SHOVE_EVERY)
    if phase > SHOVE_TIME:
        return
    R = d.xmat[bunny.torso].reshape(3, 3)
    sign = 1.0 if int(k) % 2 == 0 else -1.0
    force = sign * SHOVE["newtons"] * R[:, 1 if SHOVE["side"] else 0]  # along the body's own y (side) or x (forward)
    force[2] = 0.0
    lever = R @ [0.0, 0.0, SHOVE_HEIGHT] + d.xpos[bunny.torso] - d.xipos[bunny.torso]  # from the body's CoM
    d.xfrc_applied[bunny.torso, :3] = force
    d.xfrc_applied[bunny.torso, 3:] = np.cross(lever, force)


def drive(bunny, mode):
    """Give the controller its next command when it's ready for one."""
    shove(bunny)
    if mode == "stand":
        return
    if bunny.data.time < START_DELAY or bunny.pending_command is not None or bunny.status != "idle":
        return
    if mode == "hop" and bunny.controller.hops_started == 0:
        bunny.send(HopCommand(1, 1, 0))
    elif mode == "steps":
        bunny.send(HopCommand(1, 1, 0))
    elif mode == "bound":
        bunny.send(BoundCommand(hops=1000, push=1.0, **RAIBERT))


def side_camera(bunny):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = bunny.torso
    cam.distance = 1.1
    cam.azimuth = 90  # looking at the bunny's left side
    cam.elevation = -12
    return cam


def record(panels, seconds, slowmo, path, seed):
    """panels: list of (label, spring, mode, controller on?). Renders them side by side into one GIF."""
    bunnies = [make_bunny(DEFAULT_DESIGN, spring, seed, controller) for _, spring, _, controller in panels]
    renderers = [mujoco.Renderer(b.model, HEIGHT, WIDTH) for b in bunnies]
    cameras = [side_camera(b) for b in bunnies]
    frame_dt = slowmo / FPS  # sim time per frame
    frames = []
    for i in range(int(seconds / frame_dt)):
        row = Image.new("RGB", (WIDTH * len(panels), HEIGHT))
        for k, ((label, _, mode, _), b, r, cam) in enumerate(zip(panels, bunnies, renderers, cameras)):
            while b.data.time < i * frame_dt:
                drive(b, mode)
                b.step()
            # Swing the camera toward the bunny's left side, smoothed so a tumble doesn't whip it around
            target = math.degrees(b.heading()) + 90
            cam.azimuth += 0.15 * ((target - cam.azimuth + 180) % 360 - 180)
            r.update_scene(b.data, cam)
            img = Image.fromarray(r.render())
            draw = ImageDraw.Draw(img)
            fell = b.tilt_deg() > 60
            draw.text((8, 6), label, fill=(255, 255, 255))
            draw.text((8, 20), f"t {b.data.time:4.1f}s  hops {b.controller.hops_started}"
                               f"{'  FELL' if fell else ''}", fill=(255, 120, 120) if fell else (220, 220, 220))
            if slowmo < 1:
                draw.text((WIDTH - 70, 6), f"{slowmo:g}x speed", fill=(220, 220, 220))
            row.paste(img, (k * WIDTH, 0))
        frames.append(row)
    for r in renderers:
        r.close()
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=int(1000 / FPS), loop=0, optimize=True)
    print(f"Saved {path.relative_to(RESULTS_DIR.parent.parent)} ({len(frames)} frames)")


def aim_camera(viewer, bunny, camera, first):
    """Point the viewer's camera at the bunny ("chase" / "follow"); "free" leaves it alone."""
    if camera == "free":
        return
    cam = viewer.cam
    if first:
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = bunny.torso
        cam.distance, cam.elevation, cam.azimuth = 1.1, -12.0, 90.0
    if camera == "chase":  # swing toward the bunny's left side, smoothed so a tumble doesn't whip it round
        target = math.degrees(bunny.heading()) + 90
        cam.azimuth += 0.05 * ((target - cam.azimuth + 180) % 360 - 180)


def live(design, spring, mode, seed, seconds, controller=True, plots=True, camera="chase"):
    import mujoco.viewer
    from live_plots import LivePlots
    bunny = make_bunny(design, spring, seed, controller)
    name = 'BunnyController' if controller else 'BaselineController (no control laws)'
    print(f"{design}, spring {spring:g}, {mode}, robot {seed}, {name}")
    graphs = LivePlots(f"{name.split()[0]}, spring {spring:g}") if plots else None
    with mujoco.viewer.launch_passive(bunny.model, bunny.data, show_left_ui=not plots,
                                      show_right_ui=not plots) as viewer:
        with viewer.lock():
            aim_camera(viewer, bunny, camera, first=True)
        wall_start = time.time()
        while viewer.is_running() and bunny.data.time < seconds:
            drive(bunny, mode)
            bunny.step()
            if bunny.tilt_deg() > 60 and bunny.status == "idle":
                print("Fell over - resetting")
                bunny.reset_to_crouch()
                wall_start = time.time()
                if graphs:
                    graphs.falls += 1
                    graphs.reset()
            if int(bunny.data.time / bunny.model.opt.timestep) % 33 == 0:
                if graphs:
                    graphs.sample(bunny)
                    graphs.draw(viewer, bunny)
                with viewer.lock():
                    aim_camera(viewer, bunny, camera, first=False)
                viewer.sync()
                ahead = bunny.data.time - (time.time() - wall_start)
                if ahead > 0:
                    time.sleep(ahead)


PRESETS = {
    "level2": dict(seconds=1.8, slowmo=0.5, panels=[
        ("spring 7 (now)", 7.0, "hop", True),
        ("spring 13", 13.0, "hop", True),
        ("spring 13, no control laws", 13.0, "hop", False),
    ]),
    "level3": dict(seconds=7.0, slowmo=1.0, panels=[
        ("spring 7: one hop at a time", 7.0, "steps", True),
        ("spring 7: bounding", 7.0, "bound", True),
        ("spring 10: bounding", 10.0, "bound", True),
    ]),
    "before_after": None,  # built from --spring and --mode: without vs with the control laws
}


def main():
    parser = argparse.ArgumentParser(description="Watch the bunny live, or render GIFs.")
    parser.add_argument("--preset", choices=PRESETS)
    parser.add_argument("--design", default=DEFAULT_DESIGN, choices=DESIGNS)
    parser.add_argument("--spring", type=float, default=7.0, choices=SPRINGS)
    parser.add_argument("--mode", default="bound", choices=("hop", "steps", "bound", "stand"))
    parser.add_argument("--shove", type=float, default=0.0, metavar="N",
                        help="push the torso N newtons forward/backward every few seconds (try 6 with --mode stand; past ~10 it tips over)")
    parser.add_argument("--shove-side", action="store_true", help="--shove sideways (left/right) instead")
    parser.add_argument("--height", type=float, metavar="CM",
                        help="bound: hold this hop height (Raibert height loop)")
    parser.add_argument("--speed", type=float, metavar="CM_S",
                        help="bound: hold this liftoff speed (Raibert speed loop; needs --design latch_tethered_front_swing)")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--slowmo", type=float, default=1.0, help="0.5 = half speed (GIF only)")
    parser.add_argument("--seed", type=int, default=0, help="which randomized robot")
    parser.add_argument("--live", action="store_true", help="open the 3D viewer instead of writing a GIF")
    parser.add_argument("--no-plots", action="store_true", help="--live without the live graphs")
    parser.add_argument("--camera", default="chase", choices=("chase", "follow", "free"),
                        help="--live view: chase (side view, turns with it), follow (fixed direction), free")
    parser.add_argument("--no-controller", action="store_true",
                        help="BaselineController instead of BunnyController (no control laws), for a before/after")
    args = parser.parse_args()
    on = not args.no_controller
    SHOVE.update(newtons=args.shove, side=args.shove_side)
    RAIBERT.update(height_cm=args.height, speed_cm_s=args.speed)

    RESULTS_DIR.mkdir(exist_ok=True)
    if args.preset == "before_after":
        panels = [(f"spring {args.spring:g}, {args.mode}: no control laws", args.spring, args.mode, False),
                  (f"spring {args.spring:g}, {args.mode}: BunnyController", args.spring, args.mode, True)]
        name = f"watch_before_after_k{args.spring:g}_{args.mode}_robot{args.seed}.gif"
        record(panels, args.seconds, args.slowmo, RESULTS_DIR / name, args.seed)
    elif args.preset:
        p = PRESETS[args.preset]
        record(p["panels"], p["seconds"], p["slowmo"], RESULTS_DIR / f"watch_{args.preset}_robot{args.seed}.gif", args.seed)
    elif args.live:
        live(args.design, args.spring, args.mode, args.seed, args.seconds if args.seconds != 6.0 else 1e9, on,
             plots=not args.no_plots, camera=args.camera)
    else:
        label = f"spring {args.spring:g}, {args.mode}{'' if on else ', no control laws'}"
        name = f"watch_k{args.spring:g}_{args.mode}{'' if on else '_baseline'}_robot{args.seed}.gif"
        record([(label, args.spring, args.mode, on)], args.seconds, args.slowmo, RESULTS_DIR / name, args.seed)
    for name in BUILT:
        model_path(name).unlink(missing_ok=True)


if __name__ == "__main__":
    main()
