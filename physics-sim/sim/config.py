"""Design parameters for the Halo bunny simulation.

Everything you might want to tweak lives here. Units are SI (m, kg, s, rad, N)
unless the name says otherwise.

The robot: wind-and-latch rear legs, servo-driven front legs, powered down a
tether. (Older designs with direct-drive rear legs, passive front legs or a
battery were dropped; they're in git history before this cleanup.)

Body frame convention: x points forward (nose), y points left, z points up.
The torso origin is the geometric center of the body shell.
"""

from dataclasses import dataclass
from pathlib import Path

SIM_DIR = Path(__file__).parent
PROJECT_DIR = SIM_DIR.parent
BOM_CSV = PROJECT_DIR / "Halo Bunny Robot BOM (F26 - Terra Labs) - BOM.csv"
PART_MASSES_CSV = SIM_DIR / "part_masses.csv"
RESULTS_DIR = SIM_DIR / "results"


@dataclass
class BunnyDesign:
    # --- Torso shape (collision only; mass comes from part_masses.csv) ---
    torso_half_size: tuple = (0.14, 0.065, 0.055)  # ellipsoid semi-axes
    head_radius: float = 0.045
    head_pos: tuple = (0.12, 0.0, 0.05)

    # --- Rear legs (each wound by one ST3025 servo, HAL2-001) ---
    # Each rear leg is a thigh + shin whose knee is coupled to the hip, so the
    # foot slides in a straight line along the "push axis". The push axis
    # leans backward from vertical by rear_leg_tilt_deg, which aims the push
    # forward-and-up.
    rear_hip_pos: tuple = (-0.05, 0.06, -0.03)  # left hip; right is mirrored
    rear_leg_tilt_deg: float = 20.0
    thigh_length: float = 0.07
    shin_length: float = 0.07
    foot_radius: float = 0.012
    crouch_angle_deg: float = 65.0  # hip angle when crouched (folded)
    full_push_angle_deg: float = 12.0  # hip angle at full extension
    hip_range_deg: tuple = (5.0, 80.0)

    # --- Front legs ---
    # Each front leg is a sprung strut (mini RC shock, HAL2-059) with a thigh + shin
    # under it, coupled like the rear legs and swung by its own ST3025. The
    # controller (controller/legs.py) pushes with them at takeoff and levels the
    # body with them on landing, using the IMU (HAL2-008).
    front_shoulder_pos: tuple = (0.10, 0.05, -0.03)  # left; right is mirrored
    front_leg_travel: float = 0.03  # how far the shock can compress
    front_spring_stiffness: float = 1500.0  # N/m
    front_spring_damping: float = 30.0  # N·s/m
    front_thigh_length: float = 0.045
    front_shin_length: float = 0.045
    front_full_push_angle_deg: float = 12.0  # hip angle at full extension
    front_hip_range_deg: tuple = (5.0, 85.0)
    # Optional front shoulder swing (Raibert-style foot placement): another ST3025 per side
    # (HAL2-068) swings the whole front leg fore/aft about the shoulder on a printed bracket
    # (HAL2-069), which then carries the front hip servo. Angle + = foot forward.
    front_swing: bool = False
    front_swing_range_deg: tuple = (-30.0, 30.0)

    # --- Servo: Waveshare ST3025 at 12 V (datasheet values) ---
    servo_stall_torque: float = 3.92  # N·m  (40 kg·cm)
    servo_no_load_speed: float = 8.9  # rad/s (85 RPM)
    servo_stall_current: float = 4.4  # A
    servo_kp: float = 25.0  # N·m/rad, internal position loop (guess)
    servo_kd: float = 0.15  # N·m·s/rad (guess)
    servo_armature: float = 0.002  # reflected rotor inertia, kg·m² (guess)

    # --- Wind-and-latch rear drive ---
    # The servo slowly winds a stiff spring with a cord, a ratchet pawl holds the leg,
    # and tripping the pawl's sear lets the spring snap the leg at its own speed.
    # The spring is expressed as a torsional spring at the hip pushing the leg toward
    # extension; it goes slack at latch_spring_slack_angle_deg. How far it's wound
    # (which ratchet tooth) sets the hop energy.
    latch_spring_stiffness: float = 7.0  # N·m/rad at the hip
    latch_spring_slack_angle_deg: float = 12.0  # spring is slack at (and past) this hip angle
    latch_lever_radius: float = 0.03  # m, ratchet radius where the pawl catches
    ratchet_tooth_pitch_deg: float = 3.0  # one tooth per this much hip angle; the tooth sets the hop energy
    winder_torque_fraction: float = 0.6  # wind at most this fraction of servo stall torque
    # The release servos run off the PCA9685 (HAL2-011). It only updates a servo once per
    # PWM frame, so release times snap to frame boundaries: 50 Hz (its usual setting)
    # means up to 20 ms of delay, the same for both legs if fired in the same frame.
    # None = drive them straight from Pico 2 timer pins instead: each release starts
    # exactly when the controller schedules it, so per-robot timing can be trimmed.
    release_pwm_hz: float | None = 50.0

    # --- Contact ---
    foot_floor_friction: float = 0.9  # rubber sole (HAL2-039) on the floor

    # --- Simulation ---
    timestep: float = 0.0005


# Variants (stiffer springs, Pico timers) are registered by the scripts that use them
# (level2_big_hops.variant_name, controller_eval.pico) as dataclasses.replace() copies.
DESIGNS = {
    "latch_tethered_front": BunnyDesign(),
    "latch_tethered_front_swing": BunnyDesign(front_swing=True),
}
DEFAULT_DESIGN = "latch_tethered_front"


def model_path(design_name):
    return SIM_DIR / f"bunny_{design_name}.xml"
