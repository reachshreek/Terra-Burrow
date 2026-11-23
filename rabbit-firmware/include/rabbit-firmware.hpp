#pragma once

#include <cstdio>
#include <initializer_list>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


// ---- Pin map (Raspberry Pi Pico, RP2040, GP numbers) ----
//
// PWM rule: two GPIOs share each PWM slice, slice = (gpio / 2) % 8.
// Sear::trip() restarts its slice's counter, which would glitch any other servo on that
// slice for one frame (up to a ~3 ms pulse) - enough to twitch an armed sear open.
// So every sear gets a slice to itself; its partner pin is only ever a switch input.
// Winders never reset counters, so they share slices with each other.
//
//   slice 0  GP0  GP1    free (UART0 debug)     | GP16 GP17  foot switches
//   slice 1  GP2  GP3    sear RL, latch sw RL   | GP18 GP19  arm switch, free
//   slice 2  GP4  GP5    sear RR, latch sw RR   | GP20 GP21  free
//   slice 3  GP6  GP7    sear FL, latch sw FL   | GP22       free
//   slice 4  GP8  GP9    I2C0 SDA/SCL (IMU at 0x6A, PCA9685 at 0x40)
//   slice 5  GP10 GP11   sear FR, latch sw FR
//   slice 6  GP12 GP13   winder RL, winder RR
//   slice 7  GP14 GP15   winder FL, winder FR
//   GP26 (ADC0)  battery voltage divider       GP27, GP28 (ADC1-2) free

enum Leg { REAR_LEFT, REAR_RIGHT, FRONT_LEFT, FRONT_RIGHT, NUM_LEGS };

const uint PIN_SEAR[NUM_LEGS]     = {2, 4, 6, 10};
const uint PIN_LATCH_SW[NUM_LEGS] = {3, 5, 7, 11};
const uint PIN_WINDER[NUM_LEGS]   = {12, 13, 14, 15};
const uint PIN_FOOT[2]            = {16, 17};  // rear left, rear right
const uint PIN_ARM                = 18;
const uint PIN_VBAT               = 26;        // ADC0

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
const uint8_t PCA9685_ADDR = 0x40;  // face servos

int writei2c(uint8_t device_addr, uint8_t *src, size_t len, bool nostop);
int readi2c(uint8_t device_addr, uint8_t *dst, size_t len, bool nostop);