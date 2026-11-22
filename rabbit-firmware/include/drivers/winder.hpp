#pragma once

#include "servo.hpp"
#include "switch.hpp"

// TD-8135MG winding one leg's spring, driven as continuous rotation.
// Winds until the leg's latch switch closes, then stops. Call update() every loop tick.
//
//   Idle --wind()--> Winding --switch closed--> Paying out (slack_ms) --> Latched
//                            \--timeout--> Fault
class Winder : public Servo {
    public:
        enum class State { Idle, Winding, PayingOut, Latched, Fault };

        // direction: +1 or -1, whichever way winds the spring on this side (calibrate per leg).
        // slack_ms: how long to run backwards after latching so the cord/arm doesn't hold the
        //           leg when the sear fires; 0 if the mechanism doesn't need it.
        Winder(uint pin, uint switch_pin, int direction = 1, float wind_speed = 0.6f,
               uint32_t timeout_ms = 3000, uint32_t slack_ms = 0, uint16_t neutral_us = 1500)
            : Servo(pin, 500, 2500, neutral_us), latch_sw(switch_pin), direction(direction),
              wind_speed(wind_speed), timeout_ms(timeout_ms), slack_ms(slack_ms) {}

        void init();
        void wind();              // start winding (ignored unless Idle or Fault)
        void release();           // after the sear fires: stop and go back to Idle
        void update();
        bool latchClosed() const { return latch_sw.closed(); }
        State state() const { return st; }

    private:
        Switch latch_sw;
        int direction;
        float wind_speed;
        uint32_t timeout_ms, slack_ms;
        State st = State::Idle;
        absolute_time_t since;
};
