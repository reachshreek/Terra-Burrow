#include "rabbit-firmware.hpp"

int writei2c(uint8_t device_addr, uint8_t *src, size_t len, bool nostop) { 
    return i2c_write_timeout_us(I2C_PORT, device_addr, src, len, nostop, 2000); 
}

int readi2c(uint8_t device_addr, uint8_t *dst, size_t len, bool nostop) { 
    return i2c_read_timeout_us(I2C_PORT, device_addr, dst, len, nostop, 2000); 
}