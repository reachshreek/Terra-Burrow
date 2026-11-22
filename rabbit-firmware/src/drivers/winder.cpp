#include "drivers/winder.hpp"

void Winder::init() {
    Servo::init();
    latch_sw.init();
    off();
}

void Winder::wind() {
    if (st != State::Idle && st != State::Fault) return;
    st = State::Winding;
    since = get_absolute_time();
    setSpeed(direction * wind_speed);
}

void Winder::release() {
    off();
    st = State::Idle;
}

void Winder::update() {
    latch_sw.update();
    int64_t elapsed_ms = absolute_time_diff_us(since, get_absolute_time()) / 1000;

    switch (st) {
        case State::Winding:
            if (latchClosed()) {
                st = slack_ms ? State::PayingOut : State::Latched;
                since = get_absolute_time();
                if (slack_ms) setSpeed(-direction * wind_speed);
                else off();
            } else if (elapsed_ms > timeout_ms) {
                off();
                st = State::Fault;  // jammed, stalled, or switch broken
            }
            break;
        case State::PayingOut:
            if (elapsed_ms >= slack_ms) { off(); st = State::Latched; }
            break;
        case State::Latched:
            if (!latchClosed()) st = State::Fault;  // let go without a sear trip
            break;
        default:
            break;
    }
}
