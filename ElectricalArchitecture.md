# Halo Bunny — Electrical Architecture

## System Overview

The Halo Bunny robot uses a Raspberry Pi for computer vision
and high-level processing, while a microcontroller handles
real-time motor control, sensor feedback, and jumping.

## Electrical Architecture

```mermaid
flowchart TB
    %% Controllers
    RPI["Raspberry Pi (RPI)<br/>Computer Vision / AI / Planning"]
    MCU["Raspbery Pi Pico (MCU)<br/>Real-Time Jump Control"]

    %% Power System
    BAT["Gens Ace 3S 2200mAh 35C LiPo XT60 Battery"]
    PWR["Power Distribution<br/>Fuses / Regulators / E-Stop"]

    %% Actuators
    DRIVER["Winding Motor Driver(s)"]
    WIND["1-2 Rear-Leg Winding Motors"]
    LATCHDRV["Latch Release Driver"]
    LATCH["Latch Release Servo / Solenoid"]
    FRONT["Optional Front-Leg Actuators x2"]
    EXPRESS["Ear / Face Servos"]

    %% Sensors
    IMU["Adafruit MPU-6050 IMU"]
    LIMIT["Pololu 1528 - Winding Limit Switches"]
    LATCHSW["Pololu 1528 - Latch Engagement Sensor"]
    FOOT["Pololu 1404 - Foot Contact Sensors"]

    %% Communication
    RPI <-->|USB / UART| MCU

    %% Power Connections
    BAT --> PWR
    PWR --> RPI
    PWR --> MCU
    PWR --> DRIVER
    PWR --> LATCHDRV
    PWR --> FRONT
    PWR --> EXPRESS

    %% Motor Control
    MCU -->|Motor Commands| DRIVER
    DRIVER --> WIND

    %% Latch Control
    MCU -->|Release Command| LATCHDRV
    LATCHDRV --> LATCH

    %% Other Actuation
    MCU -->|Position Commands| FRONT
    MCU -->|PWM Signals| EXPRESS

    %% Sensor Feedback
    IMU -->|I2C| MCU
    LIMIT -->|Digital Inputs| MCU
    LATCHSW -->|Digital Input| MCU
    FOOT -->|Digital Inputs| MCU

    %% Styling
    classDef computer fill:#dbeafe,stroke:#2563eb,color:#1e3a8a
    classDef power fill:#fef3c7,stroke:#d97706,color:#78350f
    classDef actuator fill:#dcfce7,stroke:#16a34a,color:#14532d
    classDef sensor fill:#f3e8ff,stroke:#9333ea,color:#581c87

    class RPI,MCU computer
    class BAT,PWR power
    class DRIVER,WIND,LATCHDRV,LATCH,FRONT,EXPRESS actuator
    class IMU,LIMIT,LATCHSW,FOOT sensor
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