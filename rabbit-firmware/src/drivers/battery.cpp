#include "drivers/battery.hpp"
#include "hardware/adc.h"
#include "rabbit-firmware.hpp"
#include <cstdio>

// fill in with correct values
constexpr float mm_reading = 1;
constexpr float initial_reading = 1;

void Battery::init() {
    adc_init();

    adc_gpio_init(PIN_VBAT);
    adc_select_input(0);
}

void Battery::select() {
    adc_gpio_init(PIN_VBAT);
    adc_select_input(0);
}

// check on harwdared
constexpr float R1 = 1;
constexpr float R2 = 1;

float Battery::voltage() {
    select();
    return (R1 + R2) / R2 * ((adc_read() * 3.3f) / 4095) * (mm_reading / initial_reading);
}

bool Battery::check() {
    float v = voltage();
    if (v <= 9.9) return true;
    if (v <= 10.5) {
        printf("getting low on battery buddy\n");
    }

    return false;
}