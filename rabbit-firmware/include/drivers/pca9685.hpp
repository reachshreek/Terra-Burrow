#pragma once

#include "pico/stdlib.h"

// PCA9685 16-channel PWM driver over I2C, set up for hobby servos (50 Hz).
// Resolution is 12 bits per 20 ms period: one tick ≈ 4.9 µs.
//
// The chip's internal oscillator is "25 MHz" but really anywhere from ~23 to ~27 MHz,
// which scales every pulse by the same error. Measure a 1500 µs pulse with the `m` command
// in the calibration firmware (src/calibrate.cpp) and pass the osc_hz it suggests.
class PCA9685 {
    public:
        PCA9685(uint8_t addr, float osc_hz = 25'000'000.0f, float freq_hz = 50.0f)
            : addr(addr), osc_hz(osc_hz), freq_hz(freq_hz) {}

        bool init();                                 // false if the chip didn't answer
        bool setPulse(uint channel, uint16_t us);    // 0-15
        bool off(uint channel);                      // no pulses: that servo goes limp
        bool allOff();

    private:
        enum Register : uint8_t {
            MODE1 = 0x00,
            MODE2 = 0x01,
            LED0_ON_L = 0x06,  // each channel has 4 registers: ON_L, ON_H, OFF_L, OFF_H
            ALL_LED_ON_L = 0xFA,
            PRESCALE = 0xFE,
        };
        bool write(uint8_t reg, const uint8_t *data, size_t len);
        bool writeChannel(uint8_t first_reg, uint16_t on, uint16_t off);

        uint8_t addr;
        float osc_hz, freq_hz;
        float us_per_tick = 0;
};
