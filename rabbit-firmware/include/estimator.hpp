#pragma once

// Mahony attitude filter: integrates the gyro, and pulls slowly toward the accel's "down"
// only while |a| is close to 1 g (not in flight, not at a landing spike).
// No magnetometer, so yaw drifts; roll and pitch don't.
class AttitudeEstimator {
    public:
        void init(const float accel[3]);                             // level q from gravity, yaw = 0
        void update(const float gyro[3], const float accel[3], float dt);  // gyro rad/s, accel any unit (m/s2), dt s

        float roll() const;   // rad
        float pitch() const;  // rad
        float yaw() const;    // rad, drifts

        float q[4] = {1, 0, 0, 0};  // w, x, y, z (body -> world)

    private:
        float kp = 2.0f;       // rad/s per unit of gravity-direction error
        float ki = 0.0f;       // learns leftover gyro bias; calibrate() already removes most of it
        float near_1g = 0.15f; // |a| within this fraction of 1 g counts as "only gravity"
        float e_int[3] = {0, 0, 0};
};
