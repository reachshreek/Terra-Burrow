#pragma once

#include "hardware/i2c.h"

const uint8_t IMU_ADDR = 0x6A;

class IMU {
    public:
        IMU() = default;

        enum Register : uint8_t {
            CTRL1_XL = 0x10,
            CTRL2_G = 0x11,
            CTRL3_C = 0x12,
            WHOAMI = 0x0F,
            GYRO_X = 0x22,
            ACCEL_X = 0x28,
            STATUS_REG = 0x1E
        };

        bool init();
        bool readRaw();
        bool dataReady();

        float gyro_raw[3];
        float accel_raw[3];

    private:
        bool whoami();
};