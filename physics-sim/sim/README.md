# Halo bunny hop simulation

MuJoCo model of the Halo bunny, built from the BOM, for checking whether it
can hop, how far it goes, how well it steers, and whether it lands upright.

## Setup

```sh
python3 -m venv .venv
.venv/bin/pip install -r sim/requirements.txt
```

## Files

| File | What it does |
|---|---|
| `part_masses.csv` | Mass and position for every BOM line (keyed by BOM ID). **Most masses are estimates.** Replace them with weighed values as parts arrive. |
| `config.py` | Geometry, servo specs, spring and linkage values. Also the named designs: `bom` (as it stands), `improved` (servo + spring assist), `latch` (wind-and-latch), `latch_tethered` (wind-and-latch on tether power), `latch_tethered_front` (same, plus servo-driven front legs). |
| `build_model.py` | BOM + part masses + config → `bunny_<design>.xml`. Warns if a BOM line has no mass entry. |
| `servo.py` | ST3025 model: position control limited by the torque-speed line. |
| `legs.py` | Rear-leg drivers: direct servo drive, or wind-and-latch (cord winds the spring, ratchet pawl holds it, release servo fires it). Also spring and pawl sizing. |
| `front_legs.py` | Servo-driven front legs (`front_drive="servo"`) and their controller: push at takeoff, then an IMU roll/pitch PD loop that levels the body on landing. Sees only a modeled MPU-6050 (noise, bias, mounting error, range) and 12-bit encoders, at 200 Hz with one loop of bus latency. |
| `hop.py` | Runs one hop and reports distance, direction, turn, flight time, tilt and current. |
| `sweep.py` | Hop map: every command × randomized uncertainty → `results/hop_map_<design>.{csv,png}` |
| `view.py` | 3D viewer: the bunny wanders on its tether and U-turns before the cable runs out. Picks its hops from the design's hop map, so run `sweep.py` for that design first. |

## Commands

```sh
.venv/bin/python sim/build_model.py                              # rebuild after editing the BOM, masses or config
.venv/bin/python sim/hop.py --design latch_tethered              # one straight hop + spring/pawl sizing
.venv/bin/python sim/hop.py --design latch_tethered 1.0 0.7 -0.02  # left push, right push, right-leg delay (s)
.venv/bin/python sim/sweep.py --design latch_tethered            # hop map (~30 s)
.venv/bin/mjpython sim/view.py --design latch_tethered --tether 2.5  # watch it (macOS needs mjpython)
```

## Steering knobs

- **push** (0–1):
  - Servo designs: how far each hind leg extends. Below ~0.7 shortens the hop; above that barely matters.
  - Latch designs: the fraction of full spring energy, set by which ratchet tooth the leg is wound to.
- **right_delay**: positive fires the left leg first and the hop veers right. Latch designs: ±20 ms ≈ ±16° of travel direction and ±25° of body turn. Beyond ±30 ms it starts falling over.
- Firing only one leg flips the bunny.

## Results so far (straight hop, full push)

| Design | Mass | Rise | Forward | Usable cone |
|---|---|---|---|---|
| `bom` | 1.61 kg | 3 cm | 5 cm | ±23°, 4–6 cm |
| `improved` | 1.61 kg | 5 cm | 16 cm | −17° to +20°, 5–16 cm |
| `latch` | 1.72 kg | 14 cm | 35 cm | ±19°, 13–35 cm |
| `latch_tethered` | 1.44 kg | 16 cm | 39 cm | ±19°, 9–39 cm |
| `latch_tethered_front` | 1.74 kg | 12 cm | 33 cm | −25° to +21°, 10–32 cm |

Active front legs (`latch_tethered_front`, two more ST3025s, about 300 g heavier):

- Lands upright in 88% of all randomized sweep trials, against 72% for `latch_tethered`. The gain is in
  the extreme commands: ±40 ms delays and lopsided pushes that flip the passive bunny mostly land now.
- Most of that comes from the front push at takeoff, not the leveling loop. Without the push it drops to
  ~68% on the same test set. Leveling mostly shows up as the body settling level (≈0.3° vs 1.5°).
- Costs: 6 cm shorter straight hop (weight), and about half the body turn per hop (±17° vs ±37°),
  because both front feet pushing together resist yaw. The straight full-push hop is 90% upright, not 100%.
- Fewer commands pass the "usable" bar (30 vs 38), so it is not a clear win as tuned. Next things to try:
  front push timed per side to help turns, and randomizing the front servos in the sweep too.

Proposed BOM rows for the latch designs are in `../BOM changes - wind and latch.csv`.

## Things the model does not include yet

- Cable drag. The tether is modeled as a rope that only stops the bunny at its full length.
- Head and ear servo motion, which is treated as fixed mass.
- Sensor noise for the rear legs and steering. Only `front_legs.py` uses modeled sensors; the rear-leg
  servo loops use exact joint angles, and `view.py` steers from the true body position and heading.
- Gearbox backlash and compliance. Bus-servo latency is modeled only as ±5 ms random jitter in the sweep.
- Real contact stiffness and damping for the rubber feet. Tune these against slow-motion video of a one-leg test.
