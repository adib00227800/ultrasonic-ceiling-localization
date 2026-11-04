# Ceiling-Bounce 3D Indoor Localization

A low-cost 3D indoor positioning system that tracks a moving object using **one ultrasonic transmitter and four ground-level receivers**, exploiting **reflection off the ceiling** to cover the whole workspace. A downward-facing **laser rangefinder** measures the object's height in real time, so the system keeps working when the object is lifted, moves under a table, or operates beneath ceilings of different heights.

Developed for the *Measurement Systems* course (Phase 2), Faculty of Mechanical Engineering, University of Tehran.

---

## Key ideas

### 1. Wide coverage with minimal hardware
A single HC-SR04 ultrasonic sensor has a narrow field of view (roughly 15°). Covering even a 1 m × 1 m area by direct line of sight would require many sensors. Here the transmitter on the moving object fires **upward**; the wave **reflects off the ceiling** and spreads over the floor, where only **4 receivers** at the corners pick it up. The ceiling effectively widens each sensor's coverage from a narrow cone to the whole room.

### 2. Automatic height compensation with a laser
A VL53L0X (GY-53) time-of-flight laser mounted under the object measures its height above the floor. This value is fed directly into the geometry, so:
- the object can be lifted off the ground,
- the ceiling height can differ between environments,
- the object can pass under obstacles (e.g. a table) and still be localized correctly.

---

## How it works

1. The transmitter (HC-SR04, ~40 kHz) on the moving object emits a pulse toward the ceiling.
2. The pulse reflects and arrives at the floor receivers; each receiver's echo time is measured on an Arduino Mega 2560.
3. Time of flight is converted to path length using the speed of sound (≈ 343 m/s at 20 °C).
4. The laser gives the object's height; combined with the ultrasonic path lengths, this yields the horizontal radii to each receiver.
5. **Trilateration** (least squares) from the receivers gives the object's (x, y).
6. Results are streamed over serial and visualized live in 3D.

---

## Repository structure

```
.
├── arduino/
│   └── ultrasonic_laser_tracker/   # Firmware: 5x HC-SR04 + VL53L0X, CSV over serial
├── matlab/
│   ├── simulation/                 # ToF / IMU / Kalman / complementary comparison
│   └── live_gui/                   # Real-time 3D visualization + robust trilateration
├── images/                         # Setup photos and result plots
└── README.md
```

## Hardware

| Part | Qty | Role |
|---|---|---|
| HC-SR04 ultrasonic module | 1 transmitter + 4 receivers | Time-of-flight ranging |
| VL53L0X / GY-53 laser module | 1 | Height above floor |
| Arduino Mega 2560 | 1 | Timing and serial output |

## Getting started

**Firmware**
1. Install the `Adafruit_VL53L0X` library.
2. Open `arduino/ultrasonic_laser_tracker/ultrasonic_laser_tracker.ino` in the Arduino IDE, check the trig/echo pins, and upload to an Arduino Mega 2560.
3. The board prints one CSV line per measurement cycle (5 ultrasonic distances + laser height, in cm) at 9600 baud.

**Live visualization (MATLAB)**
1. Set the serial port in `matlab/live_gui/UltrasonicTrackerApp_Debug.m` (baud rate must match the firmware).
2. Run `UltrasonicTrackerApp_Debug` to see the parsed distances, the estimated (x, y, z), and the 3D scene.

**Simulation (MATLAB)**
Run `matlab/simulation/sensor_fusion_simulation.m`. It requires the Optimization Toolbox (`lsqnonlin`).

## Simulation results

Setup: 2 × 2 × 2 m cube, four floor sensors, 3D sinusoidal trajectory (10 s, 0.05 s step), Gaussian noise on distance and IMU measurements. Position RMS error in metres, for IMU noise = 0.02:

| ToF noise | ToF only | IMU only (drift) | Kalman | Complementary |
|---|---|---|---|---|
| 0.01 | 0.0187 | 0.0764 | **0.0160** | 0.0410 |
| 0.03 | 0.0651 | 0.1081 | **0.0413** | 0.0458 |
| 0.06 | 0.1157 | 0.1184 | **0.0505** | 0.0518 |

IMU-only integration drifts; fusing it with ToF measurements via a Kalman filter gives the lowest error at every tested noise level, while the complementary filter is a lighter alternative for low-power boards. Exact values vary between runs because the noise is random.

> Note: IMU fusion (Kalman / complementary filters) is evaluated in simulation. The current hardware prototype uses ultrasonic + laser sensing.

## Error sources and mitigation

- **Hardware:** HC-SR04 resolution and noise; laser instability on dark or angled surfaces.
- **Environmental:** speed of sound varies by ~0.6 m/s per °C; ceiling roughness causes multipath. Mitigation: temperature compensation, first-arrival detection.
- **Geometry (GDOP):** symmetric receiver placement at the corners minimizes error amplification.

## Future work

- Temperature-compensated speed of sound
- Real-time Kalman filtering on the hardware, adding an IMU
- Coded signals (chirp/PRN) to reject multipath
- Quantitative experimental evaluation against a ground-truth reference

## Authors

Soheil Dehghani, Seyed Mohammad Hossein Emami, Adib Saffari
Supervisor: Dr. Ayati · University of Tehran, 2025

## License

MIT
