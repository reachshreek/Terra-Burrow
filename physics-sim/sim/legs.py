"""Sizing helpers for the wind-and-latch rear legs.

The leg logic itself is in controller/legs.py, and the mechanism (spring,
ratchet, cord, release servo) is modeled in hardware.py.
"""

import math


# ---------------------------------------------------------------------------
# Sizing helpers for the wind-and-latch mechanism
# ---------------------------------------------------------------------------

def latch_spring(design, leg="rear"):
    """(stiffness N·m/rad, crouch rad, slack rad) of a wind-and-latch leg: "rear", or "front" on front_drive="latch"."""
    if leg == "front":
        from build_model import front_stance_angle  # here: build_model imports this module
        return (design.front_latch_spring_stiffness, front_stance_angle(design),
                math.radians(design.front_full_push_angle_deg))
    return (design.latch_spring_stiffness, math.radians(design.crouch_angle_deg),
            math.radians(design.latch_spring_slack_angle_deg))


def tooth_pitch(design, leg="rear"):
    """Ratchet tooth pitch (rad). A pitch spanning the whole wind (bunny_v2) is one latch point, on every leg."""
    _, crouch, slack = latch_spring(design, leg)
    rear_span = math.radians(design.crouch_angle_deg - design.latch_spring_slack_angle_deg)
    pitch = math.radians(design.ratchet_tooth_pitch_deg)
    return crouch - slack if pitch >= rear_span - 1e-9 else pitch


def peak_spring_torque(design, leg="rear"):
    """Hip torque from the spring when fully wound into the crouch."""
    k, crouch, slack = latch_spring(design, leg)
    return k * (crouch - slack)


def winder_gear_ratio(design, leg="rear"):
    """Servo turns per hip turn, chosen so full wind needs only a fraction of stall torque."""
    usable_servo_torque = design.winder_torque_fraction * design.servo_stall_torque
    return max(1.0, peak_spring_torque(design, leg) / usable_servo_torque)


def winding_report(design, push=1.0, leg="rear"):
    """Energy, forces and wind-up time for one leg at a given push."""
    gear = winder_gear_ratio(design, leg)
    k, crouch, slack = latch_spring(design, leg)
    full_stretch = crouch - slack
    stretch = math.sqrt(push) * full_stretch  # push is the fraction of full-wind energy

    # Wind from slack to full stretch. At each step the servo's speed drops as
    # the spring pushes back harder (the torque-speed line).
    steps = 400
    wind_time = 0.0
    for i in range(steps):
        hip_angle_step = stretch / steps
        hip_torque = k * (i + 0.5) * hip_angle_step
        servo_torque = hip_torque / gear
        servo_speed = design.servo_no_load_speed * (1 - servo_torque / design.servo_stall_torque)
        wind_time += hip_angle_step * gear / servo_speed

    hip_torque = k * stretch
    return {
        "energy_j": 0.5 * k * stretch**2,
        "peak_hip_torque_nm": hip_torque,
        "pawl_force_n": hip_torque / design.latch_lever_radius,
        "winder_gear_ratio": gear,
        "peak_servo_torque_nm": hip_torque / gear,
        "wind_time_s": wind_time,
    }
