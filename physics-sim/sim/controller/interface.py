"""What goes in and out of the controller, and the commands it takes.

Units: rad, rad/s, m, s, m/s². Body frame: x forward (nose), y left, z up.
"""

import math
from dataclasses import dataclass, field

import numpy as np

CONTROL_RATE_HZ = 200.0
DT = 1.0 / CONTROL_RATE_HZ
G = 9.81
SIDES = ("left", "right")


@dataclass
class HopCommand:
    """One hop, from standing still."""
    push_left: float = 1.0  # 0-1. Latch drive: fraction of full spring energy. Servo drive: how far the leg swings.
    push_right: float = 1.0
    right_delay: float = 0.0  # s; positive = right leg fires later (the hop veers right)


@dataclass
class BoundCommand:
    """A run of hops, each launched straight out of the previous landing.

    Sent while the robot is already bounding, it takes over from the next hop without
    stopping (new hop count, push, heading).
    """
    hops: int = 10
    push: float = 1.0
    heading_deg: float | None = None  # steer: bound toward this heading (the robot's own yaw estimate,
    #                                   0 = where it faced at power-up, + = left). None = every hop straight
    distance_cm: float | None = None  # steer: each hop's length (None = what `push` gives)


@dataclass
class ServoCommand:
    goal: float  # rad, at the joint (the hardware converts through any gearing)
    torque_on: bool = True  # False lets the servo go limp; on a winder that leaves the cord slack


@dataclass
class Inputs:
    gyro: np.ndarray  # rad/s, body frame, from the MPU-6050
    accel: np.ndarray  # m/s², body frame (reads +1 g up when sitting still)
    servo_position: dict  # servo name -> rad at the joint, as last read back over the bus (None before the first read)
    foot_switch: dict  # "left"/"right" -> True when that rear foot is pressed (HAL2-009)
    command: object = None  # a HopCommand or BoundCommand on the tick it's sent, else None


@dataclass
class Outputs:
    servo: dict  # servo name -> ServoCommand
    release: dict  # "left"/"right" -> True to trip that leg's sear (latch drive only)
    release_delay: dict  # "left"/"right" -> s after this tick to start that trip (sub-tick timing; a PCA9685 still waits for its next frame)
    status: str  # "idle", "starting" (waiting for the first attitude estimate), "winding", "settling", "hopping"
    hops_started: int  # count since power-up
    roll: float  # rad, + = left side up
    pitch: float  # rad, + = nose down
    yaw: float  # rad, since power-up; drifts


@dataclass
class HopPlan:
    """What one launch will actually do. BunnyController.plan_hop() fills this in."""
    push_left: float = 1.0  # 0-1, fraction of full spring energy (rounded to a ratchet tooth)
    push_right: float = 1.0
    right_delay: float = 0.0  # s, right leg's release relative to the left (+ = later); any value, scheduled within the tick
    front_push: float = 1.0  # servo front legs: 0 = no push, 1 = straighten fully
    front_delay: float = 0.0  # s from the first rear release command to the front push (negative = front pushes first)


@dataclass
class HopRecord:
    """What the robot itself measured about one hop (IMU and foot switches only).

    Handed to BunnyController.on_hop_finished() after every landing.
    """
    plan: HopPlan
    launch_pitch: float = math.nan  # rad (+ = nose down), when the push started
    launch_roll: float = math.nan
    liftoff_time: float = math.nan  # s after launch when free fall was first felt
    liftoff_pitch: float = math.nan  # rad
    liftoff_pitch_rate: float = math.nan  # rad/s (gyro y)
    liftoff_roll_rate: float = math.nan  # rad/s (gyro x)
    touchdown_time: float = math.nan  # s after launch, first impact after free fall
    touchdown_pitch: float = math.nan
    touchdown_roll: float = math.nan
    touchdown_pitch_rate: float = math.nan
    peak_g: float = 0.0  # biggest landing spike, in g
    yaw_change: float = math.nan  # rad, from the gyro, launch to settled (+ = left)
    settled_roll: float = math.nan
    settled_pitch: float = math.nan

    @property
    def flight_time(self):
        return self.touchdown_time - self.liftoff_time
