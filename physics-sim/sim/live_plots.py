"""Live graphs over the MuJoCo viewer (watch.py --live): pitch, height, heading, speed, plus a stats panel.

Each graph scrolls over the last WINDOW s and shows the true state (what a mocap rig would see) next
to what the robot itself believes (its attitude estimate), so you can watch the estimator too.

  pitch     true pitch, the robot's estimate, and true roll (deg; + = nose down / left side up)
  height    torso height above the floor (cm)
  heading   true heading and the robot's yaw estimate (deg, unwrapped; drift shows as a gap)
  speed     ground speed, smoothed over SPEED_SMOOTH s (cm/s)
"""

import collections
import math

import mujoco
import numpy as np

WINDOW = 5.0  # s of history on screen
SPEED_SMOOTH = 0.25  # s
MAX_POINTS = 1000  # an MjvFigure line holds up to 1001 points

WHITE, YELLOW, CYAN, GREY = (1.0, 1.0, 1.0), (1.0, 0.85, 0.2), (0.3, 0.8, 1.0), (0.6, 0.6, 0.6)

# title, y range (None = fit to the data), [(line name, colour)]
GRAPHS = [
    ("pitch / roll (deg)", (-60.0, 60.0), [("pitch", WHITE), ("pitch est.", YELLOW), ("roll", CYAN)]),
    ("torso height (cm)", (0.0, 40.0), [("height", WHITE)]),
    ("heading (deg)", None, [("true", WHITE), ("robot's estimate", YELLOW)]),
    ("ground speed (cm/s)", (0.0, 120.0), [("speed", WHITE)]),
]

def true_roll_pitch(bunny):
    """Same convention as the robot's estimator: roll + = left side up, pitch + = nose down."""
    R = bunny.data.xmat[bunny.torso].reshape(3, 3)
    return math.atan2(R[2, 1], R[2, 2]), -math.asin(np.clip(R[2, 0], -1, 1))


class LivePlots:
    def __init__(self, label):
        self.label = label
        self.figures = []
        for title, y_range, lines in GRAPHS:
            fig = mujoco.MjvFigure()
            fig.title = title
            fig.flg_legend = 1
            fig.flg_extend = 0
            fig.gridsize = [6, 5]
            fig.figurergba = [0.0, 0.0, 0.0, 0.55]
            fig.panergba = [0.08, 0.08, 0.1, 0.8]
            fig.xformat = "%.0f"
            fig.yformat = "%.0f"
            for i, (name, colour) in enumerate(lines):
                fig.linename[i] = name
                fig.linergb[i] = colour
            self.figures.append((fig, y_range, len(lines)))
        self.reset()

    def reset(self):
        """Clear the history (after the robot falls and gets reset)."""
        self.t = collections.deque()
        self.series = [[collections.deque() for _ in range(n)] for _, _, n in self.figures]
        self.positions = collections.deque()  # (t, x, y) for the speed estimate
        self.yaw_turns = [0.0, 0.0]  # unwrapping: last raw value and offset, true and estimate
        self.last_yaw = [None, None]
        self.start = None
        self.hops_at_start = 0
        self.max_tilt = 0.0
        self.falls = getattr(self, "falls", 0)

    def unwrap(self, k, yaw):
        if self.last_yaw[k] is not None:
            step = (yaw - self.last_yaw[k] + math.pi) % (2 * math.pi) - math.pi
            self.yaw_turns[k] += step
        else:
            self.yaw_turns[k] = yaw
        self.last_yaw[k] = yaw
        return math.degrees(self.yaw_turns[k])

    def sample(self, bunny):
        """Record one point of everything (call at the viewer's frame rate)."""
        t = bunny.data.time
        pos = bunny.torso_position()
        if self.start is None:
            self.start = (t, pos.copy())
        roll, pitch = true_roll_pitch(bunny)
        est_pitch = bunny.outputs.pitch if bunny.outputs else 0.0
        self.positions.append((t, pos[0], pos[1]))
        while self.positions[0][0] < t - SPEED_SMOOTH:
            self.positions.popleft()
        t0, x0, y0 = self.positions[0]
        speed = math.hypot(pos[0] - x0, pos[1] - y0) / (t - t0) * 100 if t > t0 else 0.0

        values = [
            [math.degrees(pitch), math.degrees(est_pitch), math.degrees(roll)],
            [pos[2] * 100],
            [self.unwrap(0, bunny.heading()), self.unwrap(1, bunny.estimated_yaw())],
            [speed],
        ]
        self.t.append(t)
        for lines, vals in zip(self.series, values):
            for line, v in zip(lines, vals):
                line.append(v)
        while self.t and (self.t[0] < t - WINDOW or len(self.t) > MAX_POINTS):
            self.t.popleft()
            for lines in self.series:
                for line in lines:
                    line.popleft()
        self.max_tilt = max(self.max_tilt, bunny.tilt_deg())

    def stats(self, bunny):
        t0, p0 = self.start
        elapsed = max(bunny.data.time - t0, 1e-9)
        moved = np.linalg.norm((bunny.torso_position() - p0)[:2]) * 100
        hops = bunny.controller.hops_started
        roll, pitch = true_roll_pitch(bunny)
        rows = [
            ("controller", self.label),
            ("status", bunny.status),
            ("time", f"{bunny.data.time:.1f} s"),
            ("hops", f"{hops}  ({hops / elapsed:.2f}/s)"),
            ("distance", f"{moved:.0f} cm  (avg {moved / elapsed:.0f} cm/s)"),
            ("pitch / roll", f"{math.degrees(pitch):+.0f} / {math.degrees(roll):+.0f} deg"),
            ("max tilt", f"{self.max_tilt:.0f} deg"),
            ("yaw drift", f"{self.series[2][1][-1] - self.series[2][0][-1]:+.1f} deg" if self.t else "-"),
            ("falls", str(self.falls)),
        ]
        return "\n".join(r[0] for r in rows), "\n".join(r[1] for r in rows)

    def draw(self, viewer, bunny):
        """Push the figures and the stats panel to the viewer (call right before viewer.sync())."""
        if not self.t:
            return
        vp = viewer.viewport
        if vp is None:
            return
        w, h = vp.width // 4, vp.height // len(self.figures)
        t = np.array(self.t)
        placed = []
        for k, ((fig, y_range, n), lines) in enumerate(zip(self.figures, self.series)):
            for i, line in enumerate(lines):
                y = np.array(line)
                fig.linepnt[i] = len(y)
                fig.linedata[i, 0:2 * len(y):2] = t - t[-1]  # seconds before now
                fig.linedata[i, 1:2 * len(y):2] = y
            if y_range is None:
                all_y = np.concatenate([np.array(line) for line in lines])
                mid, half = 0.5 * (all_y.max() + all_y.min()), max(20.0, 0.6 * (all_y.max() - all_y.min()))
                y_range = (mid - half, mid + half)
            fig.range[0] = [-WINDOW, 0.0]
            fig.range[1] = list(y_range)
            rect = mujoco.MjrRect(vp.width - w, vp.height - (k + 1) * h, w, h)
            placed.append((rect, fig))
        viewer.set_figures(placed)
        names, values = self.stats(bunny)
        viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, names, values))
