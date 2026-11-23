# Halo Bunny — Electrical Architecture

## System Overview

The Halo Bunny robot uses a Raspberry Pi for computer vision, audio
and high-level behaviour, while a Raspberry Pi Pico (RP2040) handles real-time
jump control: winding, latching, releasing, and the face servos.

Each of the four legs is wind-and-latch: a winder servo pulls the leg in
against its spring until a latch catches (confirmed by a latch switch), and
a small sear servo trips the latch to release the jump.

## Electrical Architecture

```mermaid

flowchart TB
    %% =========================
    %% POWER SYSTEM
    %% =========================
    BAT["Gens Ace 3S 2200mAh 35C<br/>LiPo XT60 Battery"]

    UV["Battery Undervoltage Cutoff<br/>Protection Circuit"]

    PD["Power Distribution"]

    REG1["Pololu 2851<br/>5V 5A Computing Regulator"]

    REG2["Servo Regulator<br/>(Model TBD)"]

    VDIV["Battery Voltage Divider"]

    %% =========================
    %% COMPUTING
    %% =========================
    RPI["Raspberry Pi 5 4GB<br/>CV / Audio / Behaviour"]

    MCU["Raspberry Pi Pico (RP2040)<br/>Real-Time Jump + Face Control"]

    SD["32GB microSD Card<br/>RPI Operating System"]

    %% =========================
    %% SENSORS
    %% =========================
    IMU["LSM6DS3-family 6-DOF IMU<br/>(confirm exact part)"]

    FOOT["Pololu 1404<br/>Rear Foot Switches x2"]

    LATCHSW["Pololu 1528<br/>Latch Engaged Switches x4"]

    MIC["ReSpeaker XVF3800<br/>4-Mic Array"]

    %% =========================
    %% SAFETY
    %% =========================
    ARM["Latch Release<br/>Arming Switch"]

    %% =========================
    %% ACTUATORS
    %% =========================
    WIND["TD-8135MG Winders x4<br/>(continuous rotation)"]

    SEAR["MG90D Sear Servos x4<br/>(latch release)"]

    PCA["PCA9685<br/>16-ch Servo Driver"]

    EARS["MG90S Ear / Eye Servos"]

    HEAD["Head Servos x2"]

    %% =========================
    %% POWER CONNECTIONS
    %% =========================
    BAT --> UV
    UV --> PD

    PD --> REG1
    REG1 --> RPI

    PD --> REG2
    REG2 --> WIND
    REG2 --> SEAR
    REG2 -->|"V+"| PCA

    PD --> VDIV

    %% =========================
    %% COMPUTER COMMUNICATION
    %% =========================
    SD -.->|"OS / Storage"| RPI

    RPI <-->|"USB Serial + Pico Power"| MCU

    MIC -->|"USB"| RPI

    %% =========================
    %% SENSOR CONNECTIONS
    %% =========================
    IMU -->|"I2C0"| MCU

    FOOT -->|"GPIO"| MCU

    LATCHSW -->|"GPIO"| MCU

    ARM -->|"GPIO"| MCU

    VDIV -->|"ADC0"| MCU

    %% =========================
    %% ACTUATOR CONTROL
    %% =========================
    MCU -->|"PWM"| WIND

    MCU -->|"PWM"| SEAR

    MCU -->|"I2C0"| PCA
    PCA --> EARS
    PCA --> HEAD

    %% =========================
    %% COLOR STYLING
    %% =========================
    classDef computer fill:#dbeafe,stroke:#2563eb,color:#1e3a8a
    classDef power fill:#fef3c7,stroke:#d97706,color:#78350f
    classDef actuator fill:#dcfce7,stroke:#16a34a,color:#14532d
    classDef sensor fill:#f3e8ff,stroke:#9333ea,color:#581c87
    classDef safety fill:#fee2e2,stroke:#dc2626,color:#7f1d1d
    classDef pending fill:#e5e7eb,stroke:#6b7280,color:#374151

    class RPI,MCU,SD computer
    class BAT,UV,REG1,REG2,VDIV power
    class IMU,FOOT,LATCHSW,MIC sensor
    class ARM safety
    class WIND,SEAR,PCA,EARS,HEAD actuator
    class PD pending

```

## System Components

| Subsystem | Component | Purpose |
|---|---|---|
| Computing | Raspberry Pi 5 | Computer vision, audio, behaviour, high-level commands |
| Control | Raspberry Pi Pico (RP2040) | Real-time winding, latching, release, face servos |
| Power | 3S LiPo Battery | Main power source |
| Power | Regulators / Fuses / E-Stop | Power distribution and electrical protection |
| Power | Battery Voltage Divider | Lets the Pico measure battery voltage (ADC0) |
| Jumping | TD-8135MG Winders x4 | Wind each leg's spring until it latches |
| Jumping | MG90D Sears x4 | Trip each leg's latch to release the spring |
| Expression | PCA9685 + MG90S servos | Ears and eyes |
| Expression | PCA9685 + head servos x2 | Head pan / tilt |
| Sensors | IMU | Attitude, liftoff and touchdown detection |
| Sensors | Latch Switches x4 | Confirm each leg is wound and latched |
| Sensors | Foot Switches x2 | Rear-foot ground contact |
| Sensors | ReSpeaker XVF3800 | Voice direction and audio (to the Pi) |
| Safety | Arming Switch | Firmware refuses to release unless armed |

## Pico Pin Map

Source of truth: `rabbit-firmware/include/rabbit-firmware.hpp`.

| Function | Pins |
|---|---|
| Sears (RL, RR, FL, FR) | GP2, GP4, GP6, GP10 (each on its own PWM slice) |
| Latch switches (RL, RR, FL, FR) | GP3, GP5, GP7, GP11 |
| I2C0 SDA / SCL (IMU, PCA9685 @ 0x40) | GP8 / GP9 |
| Winders (RL, RR, FL, FR) | GP12, GP13, GP14, GP15 |
| Foot switches (RL, RR) | GP16, GP17 |
| Arming switch | GP18 |
| Battery voltage (ADC0) | GP26 |
| Free | GP0–1 (UART0 debug), GP19–22, GP27–28 |

## Control Sequence

1. **Idle:** Check sensors, battery and the arming switch.
2. **Wind:** Run each winder until its latch switch closes, then stop it.
3. **Ready:** All four latch switches closed; wait for a jump command from the Pi.
4. **Release:** If armed, trip the four sears (rear left/right offset sets the turn).
5. **Flight:** Hold the head still; track attitude with the IMU.
6. **Landing:** Detect touchdown from the foot switches or an IMU acceleration spike.
7. **Recovery:** Settle, then wind again for the next hop.

## Important Design Notes

- Raspberry Pi handles vision, audio and behaviour; the Pico handles real-time control.
- Every sear needs its own PWM slice: releasing restarts the slice's counter, which would glitch any other servo on it.
- Servo current: four winders stalling at once can draw well over 5 A. Size the servo regulator for it, or wind the legs one or two at a time.
- The face and head servos share the servo rail; rate-limit their moves so they don't stall together.
- All grounds (battery, both regulators, Pi, Pico, servos) must be common.
- The mechanical latch must remain secure if electrical power is lost.
- An emergency stop removes actuator power but does not remove stored spring energy.
- Front-leg hip actuators are not currently planned.

**Note:** This is a functional block diagram, not a complete
electrical wiring schematic. Actual voltage rails, current
ratings, fuses, grounding, and hardware safety interlocks
must be specified before construction.
