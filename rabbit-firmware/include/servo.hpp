#pragma once

#include "pico/stdlib.h"

// Hobby PWM servo on one GPIO: 50 Hz, pulse width in µs.
// Positional (MG90D): setPulse() picks the angle.
// Continuous rotation (TD-8135MG): setSpeed() / stop(); neutral_us is where it stands still.
class Servo {
    public:
        Servo(uint pin, uint16_t min_us = 500, uint16_t max_us = 2500, uint16_t neutral_us = 1500)
            : pin(pin), min_us(min_us), max_us(max_us), neutral_us(neutral_us) {}

        void init();                     // starts the PWM output at neutral_us
        void setPulse(uint16_t us);      // clamped to [min_us, max_us]
        void setSpeed(float speed);      // -1 (full one way) .. 0 (stop) .. 1 (full the other way)
        void stop() { setPulse(neutral_us); }
        uint16_t pulse() const { return pulse_us; }

    private:
        uint pin;
        uint16_t min_us, max_us, neutral_us;
        uint16_t pulse_us = 0;
};
