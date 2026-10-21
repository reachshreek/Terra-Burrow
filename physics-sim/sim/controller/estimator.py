"""Attitude and stillness from the IMU."""

import math

import numpy as np

from .interface import DT, G

FILTER_KP = 2.0  # rad/s per unit of gravity-direction error
NEAR_1G = 0.15  # |a| within this fraction of 1 g counts as "only gravity"
STILL_GYRO = math.radians(5.0)  # rad/s on every axis
STILL_TIME = 0.2  # s of stillness before the accelerometer is trusted to correct roll/pitch
INIT_STILL_TIME = 0.1  # s of stillness, averaged, before the very first attitude estimate


def rotation_from_vector(v):
    """Rotation matrix for a rotation of |v| radians about v (Rodrigues)."""
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


class Estimator:
    """Attitude from the IMU (Mahony complementary filter) and a stillness timer.

    Roll and pitch are pulled toward the accelerometer's "down" only while the
    robot is still. Yaw has nothing to correct it, so it drifts with gyro bias.
    The filter starts once the robot has sat still for INIT_STILL_TIME, from the
    average accelerometer reading over that time. (Starting from a single sample
    can catch it mid-bounce and begin tens of degrees off.)
    """

    def __init__(self):
        self.R = None  # body -> world
        self.still_time = 0.0
        self.gyro = np.zeros(3)
        self.accel = np.array([0, 0, G])
        self.accel_sum = np.zeros(3)  # accelerometer readings during the current still stretch

    def update(self, gyro, accel):
        self.gyro, self.accel = gyro, accel
        near_1g = abs(np.linalg.norm(accel) - G) < NEAR_1G * G
        quiet = near_1g and np.all(np.abs(gyro) < STILL_GYRO)
        self.still_time = self.still_time + DT if quiet else 0.0
        if self.R is None:
            self.accel_sum = self.accel_sum + accel if quiet else np.zeros(3)
            if self.still_time >= INIT_STILL_TIME:
                self.R = self.level_from(self.accel_sum)
            return
        omega = gyro.copy()
        if self.still_time > STILL_TIME:
            measured_up = accel / np.linalg.norm(accel)
            estimated_up = self.R.T @ np.array([0, 0, 1.0])
            omega += FILTER_KP * np.cross(measured_up, estimated_up)
        self.R = self.R @ rotation_from_vector(omega * DT)
        u, _, vt = np.linalg.svd(self.R)  # re-orthonormalize so rounding doesn't build up
        self.R = u @ vt

    @staticmethod
    def level_from(accel):
        up = accel / np.linalg.norm(accel)  # world up, in the body frame
        roll = math.atan2(up[1], up[2])
        pitch = math.atan2(-up[0], math.hypot(up[1], up[2]))
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])

    def ready(self):
        return self.R is not None

    def roll_pitch(self):
        if self.R is None:
            return 0.0, 0.0
        return math.atan2(self.R[2, 1], self.R[2, 2]), -math.asin(np.clip(self.R[2, 0], -1, 1))

    def yaw(self):
        return 0.0 if self.R is None else math.atan2(self.R[1, 0], self.R[0, 0])

    def g_felt(self):
        return np.linalg.norm(self.accel) / G
