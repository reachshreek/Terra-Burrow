#pragma once

#include "pico/stdlib.h"

class Battery {
    public:
        Battery() = default;
        void init();
        bool check();
    private:
        float voltage();
        void select();
};