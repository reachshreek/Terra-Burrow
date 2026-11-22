#pragma once

#include <cstdio>
#include <initializer_list>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define SERVO_UART uart1
#define SERVO_TX 4
#define SERVO_RX 5
const uint PIN_LATCH_SERVO[2] = {14, 15};
const uint PIN_FOOT[2] = {10, 11};
const uint PIN_LATCH_SW[2] = {12, 13};
const uint PIN_LIMIT_SW[2] = {20, 21};
const uint PIN_ARM = 22;

const uint8_t WINDER_ID[2] = {1, 2};  // bus servo IDs, set with Waveshare's tool
const uint16_t LATCH_START_US = 1500;

int writei2c(uint8_t device_addr, uint8_t *src, size_t len, bool nostop);
int readi2c(uint8_t device_addr, uint8_t *dst, size_t len, bool nostop);