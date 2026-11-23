// DATASHEET: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

#include "drivers/pca9685.hpp"
#include "rabbit-firmware.hpp"

// MODE1 bits
constexpr uint8_t RESTART = 0x80, AUTO_INC = 0x20, SLEEP = 0x10;
// MODE2 bits
constexpr uint8_t OUTDRV = 0x04;   // totem-pole outputs, what servo signal lines want
// ON_H / OFF_H bit 4: channel fully on / fully off
constexpr uint16_t FULL = 0x1000;

bool PCA9685::write(uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t buf[5];  // register + up to 4 data bytes
    buf[0] = reg;
    for (size_t i = 0; i < len; i++) buf[i + 1] = data[i];
    return writei2c(addr, buf, len + 1, false) == (int)(len + 1);
}

bool PCA9685::writeChannel(uint8_t first_reg, uint16_t on, uint16_t off) {
    uint8_t d[4] = {(uint8_t)on, (uint8_t)(on >> 8), (uint8_t)off, (uint8_t)(off >> 8)};
    return write(first_reg, d, 4);
}

bool PCA9685::init() {
    // The prescaler can only be written while asleep (datasheet 7.3.5).
    uint8_t prescale = (uint8_t)(osc_hz / (4096.0f * freq_hz) + 0.5f) - 1;  // 121 for 50 Hz
    us_per_tick = (prescale + 1) * 1e6f / osc_hz;                          // actual, after rounding

    uint8_t v;
    v = SLEEP;    if (!write(MODE1, &v, 1)) return false;
    v = prescale; if (!write(PRESCALE, &v, 1)) return false;
    v = OUTDRV;   if (!write(MODE2, &v, 1)) return false;
    v = AUTO_INC; if (!write(MODE1, &v, 1)) return false;  // wake up
    sleep_us(500);                                         // oscillator needs 500 µs
    v = AUTO_INC | RESTART;
    if (!write(MODE1, &v, 1)) return false;

    return allOff();  // start limp until each servo is told where to go
}

bool PCA9685::setPulse(uint channel, uint16_t us) {
    if (channel > 15) return false;
    uint16_t ticks = (uint16_t)(us / us_per_tick + 0.5f);
    if (ticks > 4095) ticks = 4095;
    return writeChannel(LED0_ON_L + 4 * channel, 0, ticks);  // high from tick 0 to `ticks`
}

bool PCA9685::off(uint channel) {
    if (channel > 15) return false;
    return writeChannel(LED0_ON_L + 4 * channel, 0, FULL);
}

bool PCA9685::allOff() {
    return writeChannel(ALL_LED_ON_L, 0, FULL);
}
