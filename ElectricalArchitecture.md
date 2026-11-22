# Halo Bunny — Electrical Architecture

## System Overview

The Halo Bunny robot uses a Raspberry Pi for computer vision
and high-level processing, while a microcontroller handles
real-time motor control, sensor feedback, and jumping.

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

    REG2["5V 5A Servo Regulator<br/>(Model TBD)"]

    %% =========================
    %% COMPUTING
    %% =========================
    RPI["Raspberry Pi 5 4GB<br/>CV/AI/Planning"]

    MCU["Raspberry Pi Pico 2<br/>Real-Time Jump Control"]

    SD["32GB microSD Card<br/>RPI Operating System"]

    %% =========================
    %% SENSORS
    %% =========================
    IMU["Adafruit LSM6DSO32<br/>6-DOF IMU"]

    FOOT["Pololu 1404<br/>Foot Contact Switches x2"]

    HOME["Pololu 1528<br/>Winding Home Switch"]

    LIMIT["Pololu 1528<br/>Fully Wound Limit Switch"]

    LATCHSW["Pololu 1528<br/>Latch Engaged Switch"]

    %% =========================
    %% SAFETY
    %% =========================
    ARM["Independent Latch Release<br/>Arming Switch"]

    %% =========================
    %% ACTUATORS - TBD
    %% =========================
    DRIVER["Winding Motor Driver(s)"]

    WIND["1-2 Rear-Leg<br/>Winding Motors"]

    LATCHDRV["Latch Release<br/>Driver / Interface"]

    LATCH["Latch Release<br/>Servo / Solenoid"]

    FRONTDRV["Front-Leg Actuator<br/>Driver / Interface"]

    FRONT["Optional Front-Leg<br/>Actuators x2"]

    EXPRESS["Ear / Face Servos"]

    %% =========================
    %% POWER CONNECTIONS
    %% =========================
    BAT --> UV
    UV --> PD

    PD --> REG1
    REG1 --> RPI

    PD --> REG2
    REG2 --> EXPRESS

    PD --> DRIVER
    PD --> FRONTDRV

    REG2 -.->|"If 5V Servo Latch"| LATCHDRV
    PD -.->|"If Solenoid - Matched Supply"| LATCHDRV

    %% =========================
    %% COMPUTER COMMUNICATION
    %% =========================
    SD -.->|"OS / Storage"| RPI

    RPI <-->|"USB Serial + Pico Power"| MCU

    %% =========================
    %% SENSOR CONNECTIONS
    %% =========================
    IMU -->|"I2C"| MCU

    FOOT -->|"Digital GPIO"| MCU

    HOME -->|"Digital GPIO"| MCU

    LIMIT -->|"Digital GPIO"| MCU

    LATCHSW -->|"Digital GPIO"| MCU

    %% =========================
    %% ACTUATOR CONTROL
    %% =========================
    MCU -->|"Motor Commands"| DRIVER
    DRIVER --> WIND

    MCU -->|"Release Command"| LATCHDRV
    ARM -->|"Hardware Release Enable"| LATCHDRV
    LATCHDRV --> LATCH

    MCU -->|"Position Commands"| FRONTDRV
    FRONTDRV --> FRONT

    MCU -->|"PWM Signals"| EXPRESS

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
    class BAT,UV,REG1,REG2 power
    class IMU,FOOT,HOME,LIMIT,LATCHSW sensor
    class ARM safety
    class DRIVER,WIND,LATCHDRV,LATCH,FRONTDRV,FRONT,EXPRESS actuator
    class PD pending

```

## System Components

| Subsystem | Component | Purpose |
|---|---|---|
| Computing | Raspberry Pi | Computer vision, AI, high-level planning |
| Control | Microcontroller | Real-time motor and sensor control |
| Power | 3S LiPo Battery | Main power source |
| Power | Regulators / Fuses / E-Stop | Power distribution and electrical protection |
| Jumping | Winding Motor | Compresses springs to store energy |
| Jumping | Motor Driver | Controls winding motor |
| Jumping | Latch Actuator | Releases stored spring energy |
| Movement | Front-Leg Actuators | Optional landing and posture control |
| Expression | Ear / Face Servos | Animatronic movement |
| Sensors | IMU | Detects acceleration and rotation |
| Sensors | Limit Switches | Detect winding limits |
| Sensors | Latch Sensor | Confirms latch engagement |
| Sensors | Foot Sensors | Detects ground contact |

## Control Sequence

1. **Idle:** Check sensors and safety conditions.
2. **Wind:** Activate winding motors to compress springs.
3. **Latch:** Confirm the mechanical latch is engaged.
4. **Ready:** Wait for an authorized jump command.
5. **Release:** Activate latch release actuator.
6. **Flight:** Monitor orientation using the IMU.
7. **Landing:** Detect foot contact.
8. **Recovery:** Reset for the next jump.

## Important Design Notes

- Raspberry Pi handles high-level computing.
- Microcontroller handles real-time control.
- Motor drivers supply actuator current.
- Voltage regulators provide appropriate supply voltages.
- Sensors send feedback to the microcontroller.
- The mechanical latch must remain secure if electrical power is lost.
- An emergency stop removes actuator power but does not remove stored spring energy.

**Note:** This is a functional block diagram, not a complete
electrical wiring schematic. Actual voltage rails, current
ratings, fuses, grounding, and hardware safety interlocks
must be specified before construction.