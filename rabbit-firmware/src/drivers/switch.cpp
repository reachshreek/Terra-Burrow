#include "drivers/switch.hpp"

void Switch::init() {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (active_low) gpio_pull_up(pin);
    else gpio_pull_down(pin);
    sleep_us(10);  // let the pull settle before the first read
    raw = state = readRaw();  // start in whatever state it's really in, no fake edge at boot
    raw_since = get_absolute_time();
}

void Switch::update() {
    just_pressed = just_released = false;
    absolute_time_t now = get_absolute_time();

    bool r = readRaw();
    if (r != raw) { raw = r; raw_since = now; }  // still bouncing: restart the timer

    if (raw != state && absolute_time_diff_us(raw_since, now) >= debounce_us) {
        state = raw;
        just_pressed = state;
        just_released = !state;
    }
}
