#include "rabbit-firmware.hpp"
#include "drivers/imu.hpp"
#include "drivers/servo.hpp"
#include <hardware/i2c.h>

int GYRO_BIAS[3] = {0, 0, 0};

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
    float gyro_r[3], accel_ms2[3];

    sleep_ms(500);  // let USB and the servos come up
    bool imu_ok = imu.init();
    printf("rabbit-firmware up, imu %s\n", imu_ok ? "ok" : "MISSING");

    // Servo test: slow forward 1 s, stop, slow reverse 1 s, stop. Runs once at boot.
    Servo test_servo(16);
    test_servo.init();
    sleep_ms(1000);
    printf("servo test: forward\n");
    test_servo.setSpeed(0.2f);
    sleep_ms(1000);
    test_servo.stop();
    sleep_ms(1000);
    printf("servo test: reverse\n");
    test_servo.setSpeed(-0.2f);
    sleep_ms(1000);
    test_servo.stop();
    printf("servo test: done\n");

    while (true) {
        if (imu_ok && imu.dataReady()) imu.readRaw();

        for (int i = 0; i < 3; i++) {
            gyro_r[i] = (imu.gyro_raw[i] - GYRO_BIAS[i]) * 0.070f * 0.0174533f;
            accel_ms2[i] = imu.accel_raw[i] * 0.000488f * 9.80665f;
        }

        sleep_ms(20);
    }
}
