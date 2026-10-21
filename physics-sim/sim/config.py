"""Design parameters for the Halo bunny simulation.

Everything you might want to tweak lives here. Units are SI (m, kg, s, rad, N)
unless the name says otherwise.

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
    tail_pos: tuple = (-0.14, 0.0, 0.0)  # where the tether attaches

    # --- Rear legs (each driven by one ST3025 servo, HAL2-001) ---
    # Each rear leg is a thigh + shin whose knee is coupled to the hip, so the
    # foot slides in a straight line along the "push axis". The push axis
    # leans backward from vertical by rear_leg_tilt_deg, which aims the push
    # forward-and-up.
    rear_hip_pos: tuple = (-0.08, 0.06, -0.03)  # left hip; right is mirrored
    rear_leg_tilt_deg: float = 20.0
    thigh_length: float = 0.07
    shin_length: float = 0.07
    foot_radius: float = 0.012
    crouch_angle_deg: float = 65.0  # hip angle when crouched (folded)
    full_push_angle_deg: float = 12.0  # hip angle at full extension
    hip_range_deg: tuple = (5.0, 80.0)

    # --- Front legs (passive flexures / spring pads, HAL2-037) ---
    front_shoulder_pos: tuple = (0.08, 0.04, -0.03)  # left; right is mirrored
    front_leg_travel: float = 0.015  # how far the sprung foreleg can compress
    front_spring_stiffness: float = 1500.0  # N/m
    front_spring_damping: float = 15.0  # N·s/m

    # --- How the front legs are driven ---
    # "passive": a sprung strut (the BOM as it stands).
    # "servo":   each front leg is a thigh + shin with the same coupled knee as the
    #            rear legs, swung by its own ST3025, mounted under the sprung strut.
    #            front_legs.py pushes with them at takeoff and levels the body with
    #            them on landing, using the IMU (HAL2-008).
    front_drive: str = "passive"
    front_thigh_length: float = 0.045
    front_shin_length: float = 0.045
    front_full_push_angle_deg: float = 12.0  # hip angle at full extension
    front_hip_range_deg: tuple = (5.0, 85.0)

    # --- Servo: Waveshare ST3025 at 12 V (datasheet values) ---
    servo_stall_torque: float = 3.92  # N·m  (40 kg·cm)
    servo_no_load_speed: float = 8.9  # rad/s (85 RPM)
    servo_stall_current: float = 4.4  # A
    servo_kp: float = 25.0  # N·m/rad, internal position loop (guess)
    servo_kd: float = 0.15  # N·m·s/rad (guess)
    servo_armature: float = 0.002  # reflected rotor inertia, kg·m² (guess)
    # Hip radians per servo radian through the linkage (HAL2-035).
    # 1.0 = direct drive. >1 trades torque for speed.
    linkage_ratio: float = 1.0

    # --- Spring assist (HAL2-005, size not chosen yet) ---
    # Expressed as a torsional spring at the hip that pushes the leg toward
    # extension. 0 means no spring.
    spring_assist_stiffness: float = 0.0  # N·m/rad at the hip
    spring_assist_rest_angle_deg: float = 0.0

    # --- How the rear legs are driven ---
    # "servo": the servo swings the leg directly (the BOM as it stands).
    # "latch": wind-and-latch. The servo slowly winds a stiff spring, a latch
    #          holds the leg in the crouch, and releasing the latch lets the
    #          spring snap the leg at its own speed, with the servo out of the way.
    drive: str = "servo"
    # Latch-drive spring, expressed as a torsional spring at the hip pushing the leg
    # toward extension. It goes slack at latch_spring_slack_angle_deg. The servo
    # winds the leg into the crouch with a cord, stretching the spring, and a
    # ratchet pawl holds it. How far it's wound sets the hop energy.
    latch_spring_stiffness: float = 7.0  # N·m/rad at the hip
    latch_spring_slack_angle_deg: float = 12.0  # spring is slack at (and past) this hip angle
    latch_lever_radius: float = 0.03  # m, ratchet radius where the pawl catches
    winder_torque_fraction: float = 0.6  # wind at most this fraction of servo stall torque

    # --- Power source ---
    # "battery": the 3S LiPo and its protection parts ride on the robot.
    # "tether": power comes down the cable, and the e-stop and battery parts go away.
    power: str = "battery"

    # --- Contact ---
    foot_floor_friction: float = 0.9  # rubber sole (HAL2-039) on the floor

    # --- Tether (None = no tether) ---
    tether_length: float | None = None
    tether_anchor: tuple = (-0.3, 0.0, 0.0)

    # --- Simulation ---
    timestep: float = 0.0005


LATCH_LAYOUT = dict(
    rear_hip_pos=(-0.05, 0.06, -0.03),
    front_shoulder_pos=(0.10, 0.05, -0.03),
    front_leg_travel=0.03,
    front_spring_damping=30.0,
)

DESIGNS = {
    # The BOM as it stands: direct-drive servos, no spring, hips 8 cm behind center.
    "bom": BunnyDesign(),
    # Three changes found with the sim that roughly triple hop distance:
    #   - hips 3 cm further forward, so the push passes closer to the center of mass
    #   - spring assist of 2 N·m/rad at the hip (HAL2-005), about 5 N/mm on a 20 mm lever
    #   - a 1.5x linkage (HAL2-035) so the hip swings faster than the servo
    "improved": BunnyDesign(
        rear_hip_pos=(-0.05, 0.06, -0.03),
        spring_assist_stiffness=2.0,
        linkage_ratio=1.5,
    ),
    # Wind-and-latch rear legs, still on the battery. The front legs are also set
    # wider and given 30 mm of damped travel; in the sim that took turning hops
    # from ~95% to 100% upright with no loss of height.
    "latch": BunnyDesign(drive="latch", **LATCH_LAYOUT),
    # Same, powered down the tether instead (about 160 g lighter).
    "latch_tethered": BunnyDesign(drive="latch", power="tether", **LATCH_LAYOUT),
    # latch_tethered plus servo-driven front legs (two more ST3025s)
    "latch_tethered_front": BunnyDesign(drive="latch", power="tether", front_drive="servo", **LATCH_LAYOUT),
}


def model_path(design_name, with_tether=False):
    suffix = "_tether" if with_tether else ""
    return SIM_DIR / f"bunny_{design_name}{suffix}.xml"
