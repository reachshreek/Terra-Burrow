#include "drivers/imu.hpp"
#include "rabbit-firmware.hpp"
#include <pico/time.h>

bool IMU::init() {
    uint8_t reset[2] = {Register::CTRL3_C, 0x01};
    writei2c(IMU_ADDR, reset, 2, false);

    sleep_ms(100);

    if (!whoami()) return false;

    uint8_t xl[2] = {Register::CTRL1_XL, 0x74};  // CTRL1_XL: 833 Hz, ±4 g
    writei2c(IMU_ADDR, xl, 2, false);

    uint8_t g[2] = {Register::CTRL2_G, 0x7C};   // CTRL2_G: 833 Hz, ±2000 dps
    writei2c(IMU_ADDR, g, 2, false);

    uint8_t c[2] = {Register::CTRL3_C, 0x44};  
    writei2c(IMU_ADDR, c, 2, false);

    return true;
}

bool IMU::whoami() {
    uint8_t reg = Register::WHOAMI, who = 0;  // WHO_AM_I
    if (writei2c(IMU_ADDR, &reg, 1, true) != 1) return false;
    if (readi2c(IMU_ADDR, &who, 1, false) != 1 || who != 0x69) return false;

    return true;
}

bool IMU::dataReady() {
    uint8_t reg = Register::STATUS_REG, r = 0;
    if (writei2c(IMU_ADDR, &reg, 1, true) != 1) return false;
    if (readi2c(IMU_ADDR, &r, 1, false) != 1) return false;

    return (r & 0x03) == 0x03;
}

bool IMU::readRaw() {
    uint8_t reg = Register::GYRO_X, b[12];  // OUTX_L_G: gyro xyz then accel xyz
    if (writei2c(IMU_ADDR, &reg, 1, true) != 1) return false;
    if (readi2c(IMU_ADDR, b, 12, false) != 12) return false;
    for (int i = 0; i < 3; i++) {
        gyro_raw[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
        accel_raw[i] = (int16_t)(b[6 + 2 * i] | (b[7 + 2 * i] << 8));
    }

    return true;
}