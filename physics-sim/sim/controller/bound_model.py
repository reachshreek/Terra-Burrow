"""The steering model: "bound hop command -> how far it goes and how much the body turns".

  fit_model(design)   least-squares fit of distance and body turn from
                      results/bound_map_<design>.csv (None if that design has no bound map)
  best_command(...)   brute-force search over every command the hardware can do for
                      the one the model says comes closest to a target turn and distance
"""

import csv
import functools
import itertools
import math
from dataclasses import dataclass

import numpy as np

from .legs import LatchLegLogic

POSSIBLE_DELAYS = [d / 1000 for d in range(-20, 21, 5)]  # s, within ±20 ms
MAX_ENERGY_DIFFERENCE = 0.3  # lopsided pushes beyond this often fall over


def tooth_energy(push, design):
    """The push you ask for is rounded to a ratchet tooth; the energy fraction that tooth really stores."""
    leg = LatchLegLogic(design, "any")
    tooth = leg.tooth_for(push)
    return ((tooth - leg.slack) / (leg.crouch - leg.slack)) ** 2


def possible_pushes(design, lowest=0.4):
    leg = LatchLegLogic(design, "any")
    pushes, tooth = [], leg.crouch
    while (energy := ((tooth - leg.slack) / (leg.crouch - leg.slack)) ** 2) >= lowest:
        pushes.append(energy)
        tooth -= leg.pitch
    return pushes


def features(e_left, e_right, delay_ms):
    e_sum, e_diff = e_left + e_right, e_left - e_right
    return np.array([1.0, e_sum, e_diff, delay_ms, delay_ms ** 2, e_diff * delay_ms])


@dataclass
class Model:
    distance: np.ndarray  # cm
    turn: np.ndarray  # degrees of body turn, + = left

    def predict(self, e_left, e_right, delay_ms):
        x = features(e_left, e_right, delay_ms)
        return x @ self.distance, x @ self.turn


@functools.lru_cache(maxsize=None)
def _fit(design_name):
    from config import DESIGNS, RESULTS_DIR  # here, so the controller package doesn't need the sim to import
    path = RESULTS_DIR / f"bound_map_{design_name}.csv"
    if not path.exists():
        return None
    design = DESIGNS[design_name]
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["upright"] == "True" and abs(int(r["right_delay_ms"])) <= 30]
    X = np.array([features(tooth_energy(float(r["push_left"]), design), tooth_energy(float(r["push_right"]), design),
                           float(r["right_delay_ms"])) for r in rows])

    def fit(column):
        return np.linalg.lstsq(X, np.array([float(r[column]) for r in rows]), rcond=None)[0]

    return Model(fit("distance_cm"), fit("heading_change_deg"))


def fit_model(design):
    """The model for this design, or None if it has no bound map.

    From results/bound_map_<design>.csv (bound_map.py): hops straight out of a landing,
    measured launch to next launch. A design that differs only in how its release servos are driven uses its twin's map.
    """
    import dataclasses
    from config import DESIGNS, RESULTS_DIR
    for loose in (False, True):
        for name, d in DESIGNS.items():
            same = d == design or (loose and dataclasses.replace(d, release_pwm_hz=design.release_pwm_hz) == design)
            if same and (RESULTS_DIR / f"bound_map_{name}.csv").exists():
                return _fit(name)
    return None


def best_command(model, pushes, target_distance_cm, target_turn_deg, fixed=None, distance_weight=1.0):
    """(e_left, e_right, delay_s) the model says comes closest to the target distance and body turn.

    fixed: (e_left, e_right) to search only the delay, for teeth that are already wound.
    distance_weight: cost of 1 cm of distance error, in degrees² (0 = only the angle matters).
    """
    best, best_cost = None, math.inf
    teeth = [fixed] if fixed else itertools.product(pushes, pushes)
    for (e_left, e_right), delay in itertools.product(teeth, POSSIBLE_DELAYS):
        if abs(e_left - e_right) > MAX_ENERGY_DIFFERENCE and not fixed:
            continue
        distance, turn = model.predict(e_left, e_right, delay * 1000)
        cost = distance_weight * (distance - target_distance_cm) ** 2 + (turn - target_turn_deg) ** 2
        if cost < best_cost:
            best, best_cost = (e_left, e_right, delay), cost
    return best
