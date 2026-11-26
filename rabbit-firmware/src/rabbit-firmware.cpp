
#include "rabbit-firmware.hpp"
#include "drivers/battery.hpp"
#include "drivers/imu.hpp"
#include "drivers/servo.hpp"
#include <hardware/i2c.h>
#include <pico/time.h>

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);
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

    // while (true) {  // logic analyzer test: SDA/SCL toggle high/low every 500 ms
    //     gpio_pull_up(I2C_SDA);
    //     gpio_pull_up(I2C_SCL);
    //     sleep_ms(500);
    //     gpio_pull_down(I2C_SDA);
    //     gpio_pull_down(I2C_SCL);
    //     sleep_ms(500);
    // }

    printf("IMU init: %s\n", imu_ok ? "ok" : "fail");
    imu.calibrate();
    printf("IMU calibration done\n");

    auto next = get_absolute_time();
    uint32_t tick = 0;
    while (true) {
        if (imu_ok && imu.dataReady()) imu.readRaw();

        for (int i = 0; i < 3; i++) {
            gyro_r[i] = (imu.gyro_raw[i] - imu.gyro_bias[i]) * 0.070f * 0.0174533f;
            accel_ms2[i] = imu.accel_raw[i] * 0.000488f * 9.80665f  /* ±16 g: 0.488 mg/LSB */;
        }
        
        next = delayed_by_ms(next, 5);
        sleep_until(next);

        if (++tick % 40 == 0)  // print at 5 Hz, the loop runs at 200 Hz
        printf("gyro %+7.3f %+7.3f %+7.3f rad/s | accel %+7.2f %+7.2f %+7.2f m/s2\n",
            gyro_r[0], gyro_r[1], gyro_r[2],
            accel_ms2[0], accel_ms2[1], accel_ms2[2]);
    }

    return 0;

    Battery battery;
    battery.init();

    bool batteryFatal = false;
    while (true) {
        if (batteryFatal) {
            sleep_ms(1000);
            continue;
        }

        if (imu_ok && imu.dataReady()) imu.readRaw();

        for (int i = 0; i < 3; i++) {
            gyro_r[i] = (imu.gyro_raw[i] - imu.gyro_bias[i]) * 0.070f * 0.0174533f;
            accel_ms2[i] = imu.accel_raw[i] * 0.000488f * 9.80665f  /* ±16 g: 0.488 mg/LSB */;
        }
        
        next = delayed_by_ms(next, 5);
        sleep_until(next);
        
        batteryFatal = battery.check();
    }

    printf("SDA=%d SCL=%d\n", gpio_get(I2C_SDA), gpio_get(I2C_SCL));
    uint8_t b;
    printf("0x6A -> %d\n", i2c_read_timeout_us(I2C_PORT, 0x6A, &b, 1, false, 2000));

    while (!stdio_usb_connected()) sleep_ms(100);
    for (int addr = 0x08; addr < 0x78; addr++) {
        uint8_t b;
        if (i2c_read_timeout_us(I2C_PORT, addr, &b, 1, false, 2000) >= 0)
            printf("found device at 0x%02X\n", addr);
        
    }
    printf("scan done\n");

     while (true) {
        printf("imu status: %s\n", imu.init() ? "ok" : "fail");
        sleep_ms(1000);
    }

    // TD-8135MG winder bring-up: each winder in turn spins slow forward, stops, slow reverse, goes limp.
    Servo winders[1] = {Servo(8)};  // shortcut: temporary single servo on GP8 (I2C SDA pin), switch back to PIN_WINDER after the test
    for (auto &w : winders) { w.init(); w.off(); }

    while (!stdio_usb_connected()) sleep_ms(100);
    const float SPEED = 0.3f;  // keep it slow until the legs are unloaded and direction is known

    while (true) {
        for (int i = 0; i < 1; i++) {
            printf("winder %d (GP8): forward\n", i);
            winders[i].setSpeed(SPEED);
            sleep_ms(1000);
            printf("winder %d: stop (if it creeps, neutral_us needs trimming)\n", i);
            winders[i].stop();
            sleep_ms(1000);
            printf("winder %d: reverse\n", i);
            winders[i].setSpeed(-SPEED);
            sleep_ms(1000);
            winders[i].off();
            printf("winder %d: off\n\n", i);
            sleep_ms(1000);
        }
    }
}
