#pragma once

#include "servo.hpp"

// MG90D pulling one leg's latch pin. arm() holds the latch, trip() releases the spring.
//
// A PWM pin only takes a new pulse width at the start of its next 20 ms period, so trip()
// also restarts the slice counter to start the trip pulse right now. That restart can
// stretch the other pin on the same slice into one long (~3 ms) pulse, which could twitch
// an armed sear open - so give every sear a PWM slice to itself (see the pin map).
class Sear : public Servo {
    public:
        // armed_us / trip_us: placeholders from the sim, calibrate per servo on the bench.
        Sear(uint pin, uint16_t armed_us = 1500, uint16_t trip_us = 1550)
            : Servo(pin), armed_us(armed_us), trip_us(trip_us) {}

        void init() { Servo::init(); arm(); }
        void arm() { setPulse(armed_us); tripped = false; }
        void trip();
        // Trip at an exact time (hardware alarm, ~µs accurate). Returns false if no alarm was free.
        bool tripAt(absolute_time_t when);
        bool tripIn(uint32_t delay_us) { return tripAt(make_timeout_time_us(delay_us)); }
        void cancel();
        bool isTripped() const { return tripped; }

    private:
        static int64_t alarmFired(alarm_id_t, void *self);
        uint16_t armed_us, trip_us;
        volatile bool tripped = false;
        alarm_id_t alarm = 0;
};
