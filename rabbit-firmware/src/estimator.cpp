#include "estimator.hpp"
#include <cmath>

static const float G = 9.80665f;

void AttitudeEstimator::init(const float a[3]) {
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
    e_int[0] = e_int[1] = e_int[2] = 0;
}

void AttitudeEstimator::update(const float gyro[3], const float accel[3], float dt) {
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    float an = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (fabsf(an - G) < near_1g * G) {  // only trust accel when it's measuring gravity alone
        float ax = accel[0] / an, ay = accel[1] / an, az = accel[2] / an;

        // where q thinks "up" is, in the body frame
        float vx = 2 * (q1 * q3 - q0 * q2);
        float vy = 2 * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // error = measured x estimated
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        e_int[0] += ki * ex * dt;
        e_int[1] += ki * ey * dt;
        e_int[2] += ki * ez * dt;

        gx += kp * ex + e_int[0];
        gy += kp * ey + e_int[1];
        gz += kp * ez + e_int[2];
    }

    // q += 0.5 * q ⊗ (0, g) * dt
    float h = 0.5f * dt;
    q[0] = q0 + h * (-q1 * gx - q2 * gy - q3 * gz);
    q[1] = q1 + h * ( q0 * gx + q2 * gz - q3 * gy);
    q[2] = q2 + h * ( q0 * gy - q1 * gz + q3 * gx);
    q[3] = q3 + h * ( q0 * gz + q1 * gy - q2 * gx);

    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (float &c : q) c /= n;
}

float AttitudeEstimator::roll() const {
    return atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
}

float AttitudeEstimator::pitch() const {
    float s = 2 * (q[0] * q[2] - q[3] * q[1]);
    return asinf(s > 1 ? 1 : s < -1 ? -1 : s);
}

float AttitudeEstimator::yaw() const {
    return atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}
