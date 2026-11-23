#include "drivers/face_servo.hpp"
#include <cmath>

void FaceServo::setTarget(float p) {
    target = p > 1.0f ? 1.0f : (p < -1.0f ? -1.0f : p);
}

uint16_t FaceServo::toUs(float p) const {
    float span = p >= 0 ? max_us - neutral_us : neutral_us - min_us;
    return (uint16_t)(neutral_us + p * span + 0.5f);
}

void FaceServo::write() {
    uint16_t us = toUs(pos);
    if (us == last_us) return;  // save I2C time: only send changes
    if (pca.setPulse(channel, us)) last_us = us;
}

void FaceServo::update(float dt_s) {
    float err = target - pos;
    float step = err * fminf(1.0f, ease * dt_s);  // ease in: a fraction of the remaining distance
    float cap = max_speed * dt_s;                 // but never faster than max_speed
    if (step > cap) step = cap;
    if (step < -cap) step = -cap;
    pos += step;
    if (fabsf(target - pos) < 0.002f) pos = target;  // close enough: stop creeping
    write();
}
