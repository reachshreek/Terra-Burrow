#pragma once

#include "pico/stdlib.h"

class AttitudeEstimator {
    private:
        float kp = 2.0;  // rad/s per unit of gravity-direction error
        float near_1g = 0.15;  // |a| within this fraction of 1 g counts as "only gravity"

};