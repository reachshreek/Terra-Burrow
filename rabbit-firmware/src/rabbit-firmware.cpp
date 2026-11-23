#include "rabbit-firmware.hpp"
#include "drivers/battery.hpp"
#include "drivers/imu.hpp"
#include "drivers/servo.hpp"
#include <hardware/i2c.h>
#include <pico/time.h>

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    gpio_init(PICO_DEFAULT_LED_PIN);  // LED on = firmware is running
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    IMU imu;
    Battery battery;
    float gyro_r[3], accel_ms2[3];

    sleep_ms(500);  // let USB and the servos come up
    bool imu_ok = imu.init();
    printf("rabbit-firmware up, imu %s\n", imu_ok ? "ok" : "MISSING");

    battery.init();
    imu.calibrate();

    bool batteryFatal = false;
    auto next = get_absolute_time();
    while (true) {
        if (imu_ok && imu.dataReady()) imu.readRaw();

        for (int i = 0; i < 3; i++) {
            gyro_r[i] = (imu.gyro_raw[i] - imu.gyro_bias[i]) * 0.070f * 0.0174533f;
            accel_ms2[i] = imu.accel_raw[i] * 0.000488f * 9.80665f  /* ±16 g: 0.488 mg/LSB */;
        }
        
        next = delayed_by_ms(next, 5);
        sleep_ms(next);
        
        batteryFatal = battery.check();

        if (batteryFatal) break;
    }
}
