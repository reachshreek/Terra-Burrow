#include "drivers/sear.hpp"
#include "hardware/pwm.h"

void Sear::trip() {
    setPulse(trip_us);
    pwm_set_counter(pwm_gpio_to_slice_num(pin), 0);  // start the trip pulse now, not up to 20 ms later
    tripped = true;
}

int64_t Sear::alarmFired(alarm_id_t, void *self) {
    Sear *s = static_cast<Sear *>(self);
    s->alarm = 0;
    s->trip();
    return 0;  // one-shot
}

bool Sear::tripAt(absolute_time_t when) {
    cancel();
    alarm = add_alarm_at(when, alarmFired, this, true);  // fires now if `when` already passed
    return alarm >= 0;
}

void Sear::cancel() {
    if (alarm > 0) cancel_alarm(alarm);
    alarm = 0;
}
