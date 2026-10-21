# Halo bunny hop simulation

MuJoCo model of the Halo bunny, built from the BOM, for checking whether it
can hop, how far it goes, how well it steers, and whether it lands upright.

## Setup

```sh
python3 -m venv .venv
.venv/bin/pip install -r sim/requirements.txt
```

## The robot

One design, `latch_tethered_front` (`config.py`): wind-and-latch rear legs (an ST3025 winds a spring
through a cord, a ratchet pawl holds it, an MG90D trips the sear), two servo-driven front legs on RC
shocks, power down a tether. Earlier designs (direct-drive rear legs, spring assist, passive front
legs, battery) and the scripts that only they used were removed; they're in git history before the cleanup.

## Files

| File | What it does |
|---|---|
| `part_masses.csv` | Mass and position for every BOM line (keyed by BOM ID). **Most masses are estimates.** Replace them with weighed values as parts arrive. |
| `config.py` | Geometry, servo specs, spring, ratchet and release-servo values. |
| `build_model.py` | BOM + part masses + config → `bunny_latch_tethered_front.xml`. Warns if a BOM line has no mass entry. |
| `bunny_controller.py` | **The controller.** `BunnyController`'s control laws on top of the baseline: front-lead feedforward, bounding ramp, steered bounding. See "Controller" below. |
| `controller/` | The machinery under it, run once per 200 Hz tick as `outputs = controller.update(inputs)`. `interface.py`: Inputs, Outputs, commands, `HopPlan`, `HopRecord`. `estimator.py`: attitude filter. `legs.py`: rear latch (wind/unwind/fire/re-arm) and front-leg phase logic, including the leveling PD. `baseline.py`: `BaselineController`, the hop and bounding sequencing with no control laws, and the hooks `bunny_controller.py` overrides. `bound_model.py`: the steering model (fit from the bound map, inverted by brute force). |
| `hardware.py` | The robot's parts between controller and physics, no decisions: MPU-6050 (bias, noise, mounting error, range), ST3025s on the bus (command latency, their own encoder-based position loops via `servo.py`), MG90D release servos on the PCA9685 (PWM frames + travel time), the ratchet and pawl, foot switches. |
| `servo.py` | ST3025 model: position control limited by the torque-speed line, running on its own 12-bit encoder (differenced speed, deadband, 1 kHz loop, proportional only). The host only sees the position it reports over the bus. |
| `legs.py` | Latch sizing helpers: spring energy, pawl force, winder gearing, wind-up time. |
| `hop.py` | `Bunny` = model + hardware + controller, ticked together (`controller_class=BaselineController` for no control laws). `simulate_hop` sends one `HopCommand` and reports distance, direction, turn, flight time, tilt and current. |
| `sweep.py` | Hop map: every command × randomized uncertainty → `results/hop_map_<design>.{csv,png}` (characterization only; the controller doesn't use it). Also `make_uncertain_bunny`, the randomized robot every script uses. |
| `bound_map.py` | Bound map: randomized robots bound with a random command every hop; each hop measured launch to next launch → `results/bound_map_<design>.csv`. The steered-bounding model is fit from it. `--spring` for stiffer springs. |
| `watch.py` | Watch it: live 3D window (`--live`) or GIF, single hop / one at a time / bounding, any spring. `--no-controller` for the "before"; `--preset before_after` for both side by side. |
| `level2_big_hops.py` | Level 2: stiffer latch springs (bigger hops), with and without the control laws. → `results/level2_big_hops.txt` |
| `level3_bounding.py` | Level 3: bounding vs one hop at a time, with and without the control laws. `--spring`. → `results/level3_bounding*.txt` |
| `controller_eval.py` | Scores `bunny_controller.py` against the baseline on big single hops and bounding, with the release servos on the PCA9685 and on Pico timers. → `results/controller_eval.txt` |
| `agility_eval.py` | Agility: straight bounding speed, time to turn 90° and 180°, and a ±30° slalom, baseline vs BunnyController (steered bounding). `--spring 10`. → `results/agility_eval*.txt` |
| `firmware/` | **The robot's firmware, written as it would be flashed.** `main.py`: boot (gyro calibration), then the 200 Hz loop read → convert → `controller.update` → convert → write → sleep. `drivers.py`: raw counts/ticks/pulses/pins ↔ physics units. `robot_config.py`: the wiring and calibration sheet. Talks only to a Hal. See "Firmware dev environment" below. |
| `sim_hal.py` | The simulated robot as the firmware sees it: raw MPU-6050 counts, ST3025 encoder ticks (with occasional read timeouts), release-servo pulse widths, active-low bouncing foot switches, a µs clock whose `sleep_until_us` runs the physics, one-shot timers, and a serial link to the host. |
| `run_firmware.py` | Runs `firmware/main.py` on a simulated robot, with a host that sends it serial commands and a mocap that scores them. `--no-controller` too. |

## What the robot knows

The controller (`bunny_controller.py` + `controller/`) never reads the simulator's true state; everything
reaches it through `Inputs`. The true state is only used to move the physics and, in the scripts, to score
what happened, the way a motion-capture rig would. Each `Bunny(sensor_seed=...)` is a different physical
robot: its own IMU errors, release-servo speeds, PWM phase and command latencies.

## Commands

```sh
.venv/bin/python sim/build_model.py                               # rebuild after editing the BOM, masses or config
.venv/bin/python sim/hop.py                                       # one straight hop + spring/pawl sizing
.venv/bin/python sim/hop.py 1.0 0.7 -0.02                         # left push, right push, right-leg delay (s)
.venv/bin/mjpython sim/watch.py --spring 10 --mode bound --live   # watch bounding live (macOS needs mjpython)
.venv/bin/mjpython sim/watch.py --spring 10 --mode bound --live --no-controller   # the same, no control laws
.venv/bin/python sim/watch.py --preset before_after --spring 10 --mode bound      # side-by-side GIF
.venv/bin/python sim/sweep.py                                     # hop map (~30 s)
.venv/bin/python sim/bound_map.py --spring 10                     # bound map for a stiffer spring
.venv/bin/python sim/level2_big_hops.py                           # bigger hops (~2 min)
.venv/bin/python sim/level3_bounding.py --spring 10               # bounding (~2 min)
.venv/bin/python sim/run_firmware.py --do "hop 1 1 0" --do "steer 90 8"  # the firmware's main loop on a sim robot
```

## Steering knobs

- **push** (0–1): the fraction of full spring energy, set by which ratchet tooth the leg is wound to.
- **right_delay**: positive fires the left leg first and the hop veers right. ±20 ms ≈ ±15° of travel
  direction. Beyond ±20 ms it starts falling over. Commands go out on 5 ms controller ticks.
- Firing only one leg flips the bunny.

## Results so far (straight hop, full push)

1.74 kg, 17 cm rise, 31 cm forward. Usable cone (lands upright in ≥95% of randomized trials with direction
repeatable to ±10°): −19° to +18°, 9–31 cm, 50 of 81 commands (`results/sweep_log_latch_tethered_front.txt`).

Push levels are real ratchet teeth (3° apart), and moving to a lower tooth takes the controller a few
hundred ms (take the load on the cord, trip the sear, pay out, re-arm, reel back up). Each robot's two
release servos differ by a few ms, so its "straight" hop has its own consistent turn.

- **Release timing is the main limit on steering.** The release servos run off the PCA9685 at 50 Hz, so a
  release only happens at a 20 ms frame boundary, then the MG90D takes 14–22 ms (different per servo) to
  trip the sear. Driving them from Pico timer pins (`release_pwm_hz = None`) removes the frame wait.
- **Dead reckoning drifts 1–3 m within ~50 hops**, because real hops scatter around the hop-map average and
  gyro yaw drifts. Knowing where it is needs a localization sensor.

## Level 2: bigger hops (`level2_big_hops.py`)

Stiffer latch spring, everything else the same. 12 randomized robots × (straight, ±10 ms) per row, single
hops from power-up:

| Spring | Energy/leg | Pawl force | Wind time | Baseline: upright, rise, distance | BunnyController: upright, rise, distance |
|---|---|---|---|---|---|
| 7 N·m/rad (now) | 3.0 J | 216 N | 0.44 s | 100%, 16 cm, 31 cm | 100%, 17 cm, 31 cm |
| 10 | 4.3 J | 308 N | 0.62 s | 89%, 21 cm, 47 cm | 97%, 23 cm, 46 cm |
| 13 | 5.6 J | 401 N | 0.81 s | 61%, 27 cm, 62 cm | 67%, 28 cm, 62 cm |
| 16 | 6.8 J | 493 N | 1.00 s | 39%, 32 cm, 78 cm | 53%, 34 cm, 75 cm |
| 20 | 8.6 J | 617 N | 1.25 s | 19%, 39 cm, 96 cm | 39%, 42 cm, 95 cm |

The control laws help most from spring 10 up, but past ~13 the body pitches 40–60° in flight and neither
holds. After a few calibration hops `controller_eval.py` gets 86% at spring 13.

## Level 3: bounding (`level3_bounding.py`)

Ten straight hops, 12 randomized robots per row. Bounding relaunches as soon as the legs are wound and
both rear feet are down (30 ms). Speeds average the runs that didn't fall.

| Spring, controller | One at a time | Bounding |
|---|---|---|
| 7, baseline | 18 cm/s, 0/12 fell | 36 cm/s, 0/12 fell |
| 7, BunnyController | 18 cm/s, 0/12 fell | 31 cm/s, 0/12 fell |
| 10, baseline | 21 cm/s, 4/12 fell | 38 cm/s, 5/12 fell |
| 10, BunnyController | 25 cm/s, 2/12 fell | 35 cm/s, **1/12 fell** |
| 13, baseline | 29 cm/s, 12/12 fell | 66 cm/s, 12/12 fell |
| 13, BunnyController | 31 cm/s, 7/12 fell | 34 cm/s, 6/12 fell |

Bounding about doubles ground speed. At spring 7 the control laws cost ~15% of bounding speed (the ramp's
smaller first hops and the front lead); at 10 they're what keeps it upright. At 13 it still falls in half
the runs: going further needs hardware (more front-leg authority, a tail, lower centre of mass) or a landing
controller that does more than level.

Proposed BOM rows for the latch design are in `../BOM changes - wind and latch.csv`.

## Agility: steered bounding (`agility_eval.py`, `bound_map.py`)

`BoundCommand(heading_deg=...)` (or `STEER` over serial) bounds toward a heading (law 3 below). Each
landing: `turn = clip(heading - yaw, ±20°)`, teeth and release timing chosen from a model fitted on
bounding hops (`bound_map.py`); once the teeth are wound, the release timing is re-chosen at launch
from the yaw measured then. A new BoundCommand mid-bound takes over without stopping. Bounding hops
steer differently from standing hops (push mismatch turns the body the *other* way), hence the bound
map. The model leaves ~10° of per-hop turn scatter, which the heading loop corrects on the next hop.

12 unseen robots, same BoundCommands to both (the baseline ignores the heading):

| | spring 7, baseline | spring 7, bunny | spring 10, baseline | spring 10, bunny |
|---|---|---|---|---|
| straight bound: speed, s/hop (falls) | 37.2 cm/s, 0.81 (0) | 30.3 cm/s, 0.90 (0) | 40.2 cm/s, 1.04 (6) | 35.0 cm/s, 1.10 (1) |
| turn 90°: time (falls, gave up) | 14.3 s (0, 5) | **4.7 s** (0, 1) | 12.6 s (11, 0) | 11.1 s (1, 1) |
| turn 180°: time (falls, gave up) | 19.5 s (0, 8) | **9.5 s** (0, 1) | 12.0 s (11, 0) | 11.1 s (3, 2) |
| ±30° slalom: heading error, speed (falls) | 53° rms, 35 cm/s (0) | **31° rms**, 20 cm/s (0) | 61° rms, 39 cm/s (8) | **37° rms**, 18 cm/s (1) |

The baseline only "turns" by drifting, so its turn times are the robots that happened to drift the
right way. The slalom flips the order every 3 hops, faster than ~20°/hop can follow, so it's a stress test.

## Controller (`bunny_controller.py`)

`BaselineController` (`controller/baseline.py`) runs the same machinery with none of these laws: every
hop exactly as commanded, front legs pushing with the rear, no ramp, no steering. That's the "before" for
`--no-controller`. The machinery both share: the servos' own position loops, the attitude estimator, the
front-leg leveling PD in stance, and the rear-latch and front-leg state machines.

1. **Front-lead feedforward.** The front legs push `FRONT_LEAD` (30 ms) before the rear releases.
   Step test: upright with a 0 / 15 / 30 ms lead was 100/100/100% at spring 7, 100/100/94% at 10,
   and 31/94/75% at 13.
2. **Bounding ramp.** Bounding starts at half push and reaches full push over 3 hops.
3. **Steered bounding.** A heading loop, one correction per bound hop (see "Agility" above).

`controller_eval.py`, 12 randomized robots the maps never saw:

| Test | Baseline | `bunny_controller` |
|---|---|---|
| Big hops, spring 10, upright (PCA / Pico) | 96% / 92% | **98% / 100%** |
| Big hops, spring 13, upright (PCA / Pico) | 75% / 78% | 77% / **85%** |
| Bounding 10 hops, spring 10, runs that fell (PCA / Pico) | 6/12 / 10/12 | **1/12 / 1/12** |
| Bounding 10 hops, spring 13, runs that fell (PCA / Pico) | 12/12 / 12/12 | **8/12 / 9/12** |

Removed to keep the controller small, or because they didn't help: aiming single hops at a landing spot (a fitted feedforward; ≈2 cm / 3–4° vs ≈5 cm / 9°),
planning each bound hop at the previous launch so the winders reel straight to its teeth (0.87 → 0.82 s
a hop), keeping teeth a landing folded the leg onto, holding the rear legs straighter to catch landing
energy (the landing adds only 1–2° of fold; the front shocks take most of it), an IMU launch gate for
bounding (no fewer falls), an integral loop on liftoff pitch rate, a per-robot release-mismatch integral
term, and front-leg reach scheduled from predicted landing pitch.

`release_pwm_hz = None` in a design means the release servos run straight from Pico timer pins, so the
controller can schedule each release to sub-millisecond precision (`Outputs.release_delay`). With this
controller it didn't beat the PCA9685; it matters if you want to trim timing per robot.

Fixed along the way: the attitude estimate used to start from a single accelerometer sample, which could
catch the bunny mid-bounce and start tens of degrees off; the front legs would then "level" it over. It
now waits for 0.1 s of stillness and averages. Commands wait (`status == "starting"`) until it's ready.

## Firmware dev environment (`firmware/`, `sim_hal.py`, `run_firmware.py`)

Develop the robot code the way it runs on the robot: `firmware/main.py` owns the main loop and
only ever sees raw numbers, through a `hal` object. In the sim that's `sim_hal.SimHal`; on the
robot it would be a Pico 2 class with the same calls wrapping the real drivers.

```
firmware/main.py                      sim_hal.py (here)  /  Pico 2 Hal (robot)
  boot: calibrate gyro  ─────────────▶  hal.imu.read_raw()           MPU-6050 counts
  loop every 5 ms:                      hal.servos.read_position(id) ST3025 ticks, or None
    read + convert   (drivers.py)       hal.gpio.read(pin)           foot switch, 0 = pressed
    host command     (serial line)      hal.link.readline()/write()  USB serial
    controller.update(inputs)
    convert + write  (drivers.py) ───▶  hal.servos.write(id, ticks, torque_on)
                                        hal.pwm.set_pulse_us(ch, us) release servos
                                        hal.clock.call_later_us()    sub-tick release timing
    sleep until next tick  ──────────▶  hal.clock.sleep_until_us()   (the physics runs here)
```

What the firmware has to deal with that the old `Inputs` hid:
- The gyro has a factory offset of a few °/s: `drivers.Imu.calibrate()` averages it out at boot
  while the robot sits still, and retries if it moved. A further ±0.5 °/s creeps in as the chip
  warms up (20 s), which no boot calibration can catch.
- Servo positions are encoder ticks at the servo shaft, through each horn's zero and direction
  and the winder's gearing (`robot_config.py`). About 1 in 500 bus reads times out; the driver
  keeps the last good value.
- Foot switches are active-low and chatter for 0.5-3 ms at each make and break; the driver debounces.
- Release servos take pulse widths; the controller's sub-tick release timing becomes a one-shot timer.
- Commands arrive as text lines over serial (`HOP`, `BOUND`, `STEER`), a few ms late, and the robot
  reports `STATUS` and `HOP_DONE` lines back.

Checked against the old path (controller called directly, `hop.py`), 12 unseen robots:
bounding 10 hops at spring 10, 2/12 runs fell either way. Same robots, different random draws, so not identical.

Not modeled: how long the loop takes to run (reads and the controller take zero time here; on
the Pico, a Python controller with numpy may not fit in 5 ms: check, or port the hot parts to C),
bus bandwidth, and I2C errors. ROS 2 isn't used: the Pico can't run it, and a 200 Hz loop
belongs on the microcontroller. If you want ROS 2 on the host side later, a node that speaks
this serial protocol (or micro-ROS on the Pico) slots in where `run_firmware.Host` is.

## Things the model does not include yet

- The tether: no cable, drag or length limit is modeled.
- Spring-rate tolerance: `sweep.Uncertainty` draws ±10% per latch spring, but it isn't applied yet (the
  latch spring's rate lives in `hardware.py`, not the model), so every robot's springs are exact.
- Head and ear servo motion, which is treated as fixed mass.
- Gearbox backlash and compliance, and flex in the printed parts.
- Cord slack: after latching, the winder servo's encoder is treated as still showing the hip angle.
- The joint limit switches (HAL2-010) aren't used yet, and encoder zeros are taken as known (no homing).
- Battery or tether voltage sag under the servos' current draw.
- The servo's internal loop rate, deadband and speed filter are guesses; Feetech doesn't publish them.
- Real contact stiffness and damping for the rubber feet. Tune these against slow-motion video of a one-leg test.
