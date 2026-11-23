#pragma once

#include "pca9685.hpp"

// One face/head servo on a PCA9685 channel, with hard limits and smooth motion.
//
// Positions are -1 .. 0 .. 1, mapped to min_us .. neutral_us .. max_us. Calibrate
// min/max to where the part physically stops (ear against the head, eyelid shut), so
// nothing can ever be told to push past them.
//
// setTarget() only sets where to go; update(dt) moves toward it a little each call,
// easing in (slows near the target) and capped at max_speed, so moves never snap
// and never block the loop. The PCA9685 only outputs a new pulse every 20 ms, so
// calling update() at 50 Hz is enough.
class FaceServo {
    public:
        FaceServo(PCA9685 &pca, uint channel, uint16_t min_us, uint16_t max_us,
                  uint16_t neutral_us = 1500, float max_speed = 2.0f, float ease = 8.0f)
            : pca(pca), channel(channel), min_us(min_us), max_us(max_us),
              neutral_us(neutral_us), max_speed(max_speed), ease(ease) {}

        void init() { pos = target = 0; write(); }  // jumps straight to neutral
        void setTarget(float p);                    // clamped to -1 .. 1
        void snap(float p) { setTarget(p); pos = target; write(); }  // move now, no easing
        void update(float dt_s);
        void relax() { pca.off(channel); last_us = 0; }  // limp; next update() re-engages
        float position() const { return pos; }
        bool atTarget() const { return pos == target; }

    private:
        void write();
        uint16_t toUs(float p) const;

        PCA9685 &pca;
        uint channel;
        uint16_t min_us, max_us, neutral_us;
        float max_speed;  // full range units per second (2.0 = end to end in 1 s)
        float ease;       // 1/s: how hard it slows down near the target
        float pos = 0, target = 0;
        uint16_t last_us = 0;
};
