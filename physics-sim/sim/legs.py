"""Sizing helpers for the wind-and-latch rear legs.

The leg logic itself is in controller/legs.py, and the mechanism (spring,
ratchet, cord, release servo) is modeled in hardware.py.
"""

import math


# ---------------------------------------------------------------------------
# Sizing helpers for the wind-and-latch mechanism
# ---------------------------------------------------------------------------

def peak_spring_torque(design):
    """Hip torque from the spring when fully wound into the crouch."""
    stretch = math.radians(design.crouch_angle_deg - design.latch_spring_slack_angle_deg)
    return design.latch_spring_stiffness * stretch


def winder_gear_ratio(design):
    """Servo turns per hip turn, chosen so full wind needs only a fraction of stall torque."""
    usable_servo_torque = design.winder_torque_fraction * design.servo_stall_torque
    return max(1.0, peak_spring_torque(design) / usable_servo_torque)


def winding_report(design, push=1.0):
    """Energy, forces and wind-up time for one leg at a given push."""
    gear = winder_gear_ratio(design)
    k = design.latch_spring_stiffness
    full_stretch = math.radians(design.crouch_angle_deg - design.latch_spring_slack_angle_deg)
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
