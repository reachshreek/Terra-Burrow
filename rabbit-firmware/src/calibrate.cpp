// Servo calibration firmware: a separate program you flash instead of the main one.
// Talk to it over USB serial (e.g. `screen /dev/tty.usbmodem* 115200`), one command per line:
//
//   w <0-3>    select winder   (0 RL, 1 RR, 2 FL, 3 FR)
//   s <0-3>    select sear     (only moves while the arm switch is ON)
//   f <0-15>   select face servo on PCA9685 channel 0-15
//   1500       set the selected servo's pulse to 1500 µs (500-2500)
//   +  / -     nudge by 10 µs        +2 / -50   nudge by that many µs
//   off        stop pulses on the selected servo (goes limp)
//   p          print everything
//   m          measure the pulse on GP19 (jumper a PCA9685 output there) and suggest osc_hz
//
// Switch changes (latch x4, feet x2, arm) print as they happen, so this also tests the wiring.
//
// What to find, then copy into the constructors in the main firmware:
//   winder: the pulse where it stands completely still (neutral_us), and which way
//           (+ or - from neutral) winds the spring (direction). Unwound, latch disconnected.
//   sear:   armed_us (holds the latch) and trip_us (just past where it releases).
//           Do this with the springs UNWOUND: a sear that moves can fire a loaded leg.
//   face:   min_us / max_us where each part physically stops, and a neutral_us.
//           Come at the limits slowly with small nudges, a stalled servo gets hot fast.
//   PCA9685 clock: `f 0`, `1500`, jumper channel 0's signal pin to GP19, then `m`.
//           The PCA9685's VCC (logic) pin MUST be on 3.3 V for this: its outputs swing to
//           VCC, and 5 V on a Pico pin can kill it.

#include "rabbit-firmware.hpp"
#include "drivers/servo.hpp"
#include "drivers/switch.hpp"
#include "drivers/pca9685.hpp"
#include <cstdlib>
#include <cstring>

static const char *LEG_NAME[NUM_LEGS] = {"RL", "RR", "FL", "FR"};

static Servo winders[NUM_LEGS] = {Servo(PIN_WINDER[0]), Servo(PIN_WINDER[1]),
                                  Servo(PIN_WINDER[2]), Servo(PIN_WINDER[3])};
static Servo sears[NUM_LEGS] = {Servo(PIN_SEAR[0]), Servo(PIN_SEAR[1]),
                                Servo(PIN_SEAR[2]), Servo(PIN_SEAR[3])};
static Switch latch_sw[NUM_LEGS] = {Switch(PIN_LATCH_SW[0]), Switch(PIN_LATCH_SW[1]),
                                    Switch(PIN_LATCH_SW[2]), Switch(PIN_LATCH_SW[3])};
static Switch feet[2] = {Switch(PIN_FOOT[0]), Switch(PIN_FOOT[1])};
static Switch arm_sw(PIN_ARM);
static PCA9685 pca(PCA9685_ADDR);

static char kind = 0;      // 'w', 's', 'f', or 0 for nothing selected
static int sel = 0;
static int pulse_us = 0;   // 0 = off
static uint16_t face_us[16] = {0};
static bool pca_ok = false;

constexpr uint PIN_MEASURE = 19;  // free in the main pin map; only used by this tool
constexpr float PCA_DEFAULT_OSC = 25'000'000.0f;

// Time how long PIN_MEASURE stays high, averaged over `n` pulses. Returns 0 if no pulse
// showed up (each wait gives up after 25 ms, longer than one 20 ms servo frame).
static float measurePulse(int n) {
    uint32_t total = 0;
    for (int i = 0; i < n; i++) {
        uint32_t t = time_us_32();
        while (gpio_get(PIN_MEASURE)) if (time_us_32() - t > 25000) return 0;   // skip a pulse already in progress
        t = time_us_32();
        while (!gpio_get(PIN_MEASURE)) if (time_us_32() - t > 25000) return 0;  // wait for it to start
        uint32_t start = time_us_32();
        while (gpio_get(PIN_MEASURE)) if (time_us_32() - start > 25000) return 0;  // stuck high
        total += time_us_32() - start;
    }
    return (float)total / n;
}

static void printStatus() {
    printf("\n-- selected: %s", kind ? "" : "none");
    if (kind == 'w') printf("winder %s", LEG_NAME[sel]);
    if (kind == 's') printf("sear %s", LEG_NAME[sel]);
    if (kind == 'f') printf("face ch %d", sel);
    if (kind) printf(", pulse %s", pulse_us ? "" : "off");
    if (kind && pulse_us) printf("%d us", pulse_us);
    printf("\n   winders:");
    for (int i = 0; i < NUM_LEGS; i++) printf(" %s=%d", LEG_NAME[i], winders[i].pulse());
    printf("\n   sears:  ");
    for (int i = 0; i < NUM_LEGS; i++) printf(" %s=%d", LEG_NAME[i], sears[i].pulse());
    printf("\n   face:   ");
    for (int i = 0; i < 16; i++) if (face_us[i]) printf(" ch%d=%d", i, face_us[i]);
    printf("%s\n   latch:  ", pca_ok ? "" : " (PCA9685 not found)");
    for (int i = 0; i < NUM_LEGS; i++) printf(" %s=%s", LEG_NAME[i], latch_sw[i].closed() ? "closed" : "open");
    printf("\n   feet:    RL=%s RR=%s   arm=%s\n",
           feet[0].closed() ? "down" : "up", feet[1].closed() ? "down" : "up",
           arm_sw.closed() ? "ON" : "off");
}

// Send pulse_us to the selected servo (0 = off). Returns false if it wasn't allowed.
static bool apply() {
    if (kind == 's' && pulse_us && !arm_sw.closed()) {
        printf("arm switch is off, sear not moved\n");
        return false;
    }
    if (kind == 'w') pulse_us ? winders[sel].setPulse(pulse_us) : winders[sel].off();
    if (kind == 's') pulse_us ? sears[sel].setPulse(pulse_us) : sears[sel].off();
    if (kind == 'f') {
        if (!pca_ok) { printf("PCA9685 not found\n"); return false; }
        pulse_us ? pca.setPulse(sel, pulse_us) : pca.off(sel);
        face_us[sel] = pulse_us;
    }
    return true;
}

static void handle(char *line) {
    while (*line == ' ') line++;
    if (!*line) return;

    if ((line[0] == 'w' || line[0] == 's' || line[0] == 'f') && line[1] == ' ') {
        int n = atoi(line + 2), max = line[0] == 'f' ? 15 : NUM_LEGS - 1;
        if (n < 0 || n > max) { printf("sel must be 0-%d\n", max); return; }
        kind = line[0];
        sel = n;
        // Pick up wherever that servo already is; nothing moves until you send a pulse.
        if (kind == 'w') pulse_us = winders[n].pulse();
        if (kind == 's') pulse_us = sears[n].pulse();
        if (kind == 'f') pulse_us = face_us[n];
        printStatus();
        return;
    }
    if (!strcmp(line, "p")) { printStatus(); return; }
    if (!strcmp(line, "m")) {
        float us = measurePulse(20);
        if (!us) { printf("no pulse on GP%d (jumpered? channel on?)\n", PIN_MEASURE); return; }
        printf("measured %.1f us\n", us);
        if (kind == 'f' && pulse_us)
            printf("commanded %d us -> use PCA9685(PCA9685_ADDR, %.0f)\n",
                   pulse_us, PCA_DEFAULT_OSC * pulse_us / us);
        return;
    }
    if (!kind) { printf("select a servo first (w/s/f <n>)\n"); return; }

    int next;
    if (!strcmp(line, "off")) next = 0;
    else if (!strcmp(line, "+")) next = (pulse_us ? pulse_us : 1500) + 10;
    else if (!strcmp(line, "-")) next = (pulse_us ? pulse_us : 1500) - 10;
    else if (line[0] == '+' || line[0] == '-') next = (pulse_us ? pulse_us : 1500) + atoi(line);
    else if (line[0] >= '0' && line[0] <= '9') next = atoi(line);
    else { printf("unknown command: %s\n", line); return; }

    if (next && (next < 500 || next > 2500)) { printf("pulse must be 500-2500 us\n"); return; }
    int prev = pulse_us;
    pulse_us = next;
    if (apply()) printf("%d us\n", pulse_us);
    else pulse_us = prev;
}

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Everything starts limp: nothing moves until you select it and send a pulse.
    for (int i = 0; i < NUM_LEGS; i++) {
        winders[i].init(); winders[i].off();
        sears[i].init();   sears[i].off();
        latch_sw[i].init();
    }
    feet[0].init(); feet[1].init(); arm_sw.init();
    gpio_init(PIN_MEASURE);
    gpio_set_dir(PIN_MEASURE, GPIO_IN);
    gpio_pull_down(PIN_MEASURE);  // reads low (no pulse) when nothing is jumpered
    pca_ok = pca.init();

    while (!stdio_usb_connected()) sleep_ms(100);  // wait for the terminal so the help isn't lost
    sleep_ms(200);
    printf("\nrabbit calibrate. commands: w|s|f <n>, <us>, +, -, +<n>, -<n>, off, p, m\n");
    printStatus();

    char line[32];
    int len = 0;
    while (true) {
        // Switches: report every change.
        for (int i = 0; i < NUM_LEGS; i++) {
            latch_sw[i].update();
            if (latch_sw[i].pressed()) printf("[latch %s closed]\n", LEG_NAME[i]);
            if (latch_sw[i].released()) printf("[latch %s open]\n", LEG_NAME[i]);
        }
        for (int i = 0; i < 2; i++) {
            feet[i].update();
            if (feet[i].pressed()) printf("[foot %s down]\n", LEG_NAME[i]);
            if (feet[i].released()) printf("[foot %s up]\n", LEG_NAME[i]);
        }
        arm_sw.update();
        if (arm_sw.pressed()) printf("[arm ON]\n");
        if (arm_sw.released()) printf("[arm off]\n");

        // Serial input, one line at a time, echoed back.
        int c = getchar_timeout_us(0);
        if (c == '\r' || c == '\n') {
            if (len) { printf("\n"); line[len] = 0; handle(line); len = 0; }
        } else if ((c == 8 || c == 127) && len) {
            len--; printf("\b \b");
        } else if (c >= 32 && c < 127 && len < (int)sizeof(line) - 1) {
            line[len++] = (char)c; putchar(c);
        }

        sleep_ms(1);
    }
}
