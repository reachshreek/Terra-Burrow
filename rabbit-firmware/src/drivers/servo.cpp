#include "drivers/servo.hpp"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

void Servo::init() {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);

    // Two pins share each PWM slice (e.g. GP14 + GP15), and pwm_init() zeroes both channels,
    // so only set the slice up once or the other servo loses its pulse.
    if (!(pwm_hw->slice[slice].csr & PWM_CH0_CSR_EN_BITS)) {
        pwm_config c = pwm_get_default_config();
        pwm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / 1'000'000.0f);  // 1 count = 1 µs
        pwm_config_set_wrap(&c, 20000 - 1);                                 // 20 ms period = 50 Hz
        pwm_init(slice, &c, true);
    }
    stop();
}

void Servo::setPulse(uint16_t us) {
    if (us < min_us) us = min_us;
    if (us > max_us) us = max_us;
    pulse_us = us;
    pwm_set_gpio_level(pin, us);
}

void Servo::setSpeed(float speed) {
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;
    float span = speed >= 0 ? max_us - neutral_us : neutral_us - min_us;
    setPulse((uint16_t)(neutral_us + speed * span));
}

void Servo::off() {
    pulse_us = 0;
    pwm_set_gpio_level(pin, 0);
}
