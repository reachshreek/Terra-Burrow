#pragma once

#include "pico/stdlib.h"

// Mechanical switch on one GPIO, debounced by time: a new state only counts once the pin
// has held it for debounce_us, so it works at any loop rate. Call update() every loop tick.
//
// Default wiring: COM to GND, NO to the pin, internal pull-up (closed reads 0).
// For a switch wired to 3.3 V instead, pass active_low = false (uses a pull-down).
class Switch {
    public:
        Switch(uint pin, bool active_low = true, uint32_t debounce_us = 5000)
            : pin(pin), active_low(active_low), debounce_us(debounce_us) {}

        void init();
        void update();
        bool closed() const { return state; }
        bool pressed() const { return just_pressed; }    // closed on this update()
        bool released() const { return just_released; }  // opened on this update()

    private:
        bool readRaw() const { return gpio_get(pin) != active_low; }
        uint pin;
        bool active_low;
        uint32_t debounce_us;
        bool state = false, raw = false;
        bool just_pressed = false, just_released = false;
        absolute_time_t raw_since;
};
