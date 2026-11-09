# ultrasonic-ceiling-localization

A low-cost 3D indoor positioning system that tracks a moving object using **one upward-facing ultrasonic transmitter and four ground-level receivers**, exploiting **reflection off the ceiling** to extend the coverage of each sensor. A downward-facing **laser rangefinder** measures the object's height in real time, and the upward transmitter measures the ceiling clearance, so the system keeps working when the object is lifted or operates beneath ceilings of different heights.

Developed for the *Measurement Systems* course (Phase 2), Faculty of Mechanical Engineering, University of Tehran.

![Conceptual model](images/concept_model.jpeg)

---

## Key ideas

### 1. Extended coverage with minimal hardware

A single HC-SR04 ultrasonic sensor has a narrow beam (roughly 15°; the exact working angle must be measured experimentally). Let α be the beam half-angle, H the ceiling height and z the object height.

If receivers were mounted on the ceiling, each would cover a region of radius about (H − z)·tan α around it. Here, instead, the transmitter on the moving object fires **upward**, the wave **reflects off the ceiling**, and the floor receivers (also facing upward) pick it up. The reflected path is equivalent to a straight path from a virtual source at height 2H − z, so the usable horizontal radius per receiver becomes about (2H − z)·tan α.

The ratio of the two radii is

$$\frac{(2H - z)\tan\alpha}{(H - z)\tan\alpha} = \frac{2H - z}{H - z} \quad (0 \le z < H)$$

This equals **2 at floor level (z = 0)** and increases as the object is lifted. With the **same number of sensors** and nothing mounted on the ceiling, the coverage radius of each sensor is therefore about **2× larger near the floor** (about 4× in area). Notes:

- The ratio is per sensor. Position estimation needs the object to lie inside the beams of at least three receivers at the same time.
- The ratio grows with z, but the absolute radius (2H − z)·tan α shrinks as z grows.
- The real beam angle and coverage gain are to be confirmed experimentally (see Future work).

### 2. Automatic compensation for ceiling height and object height

The ceiling height is **not assumed to be known**, so the system works under ceilings of different heights. In every cycle:

1. The upward-facing transmitter `d5` sends a pulse and records the ceiling echo directly above it, giving the ceiling clearance $h_c$.
2. The VL53L0X (GY-53) time-of-flight laser, mounted under the object, measures the height above the floor $z$.
3. The virtual-source height is computed as $Z_{\text{eff}} = 2h_c + z$ (equivalently $2H - z$ with $H = h_c + z$).
4. The floor receivers measure the ceiling-reflected path lengths, which are converted to horizontal distances using $Z_{\text{eff}}$.

Each measured path length mixes the horizontal distance and the vertical component, so $Z_{\text{eff}}$ is needed to separate them. In principle $Z_{\text{eff}}$ could be estimated as an additional unknown, but because all receivers lie in one plane this problem is ill-conditioned; measuring it directly avoids that.

The same formula stays valid when the object operates under a lower horizontal surface (for example a table), as long as the reflected pulse still reaches the receivers; accuracy is lower in that case.

**Assumption:** the reflecting surface above the object is flat and horizontal.

---

## How it works & Mathematical Formulation

### 1. 5-Sensor Architecture

The system uses five HC-SR04 modules (`d1` to `d5`) connected to an Arduino Mega 2560. **All sensors face upward.**

- **Moving transceiver (`d5`):** mounted on the moving object. It emits the 40 kHz ultrasonic burst toward the ceiling and also records the direct vertical echo from the ceiling ($h_c$).
- **Four receivers (`d1`–`d4`):** placed at known floor coordinates $(x_i, y_i, 0)$. Their transmitters are **physically blocked**, so they only listen. Each measures the time of flight of the ceiling-reflected pulse emitted by `d5`. Because the standard HC-SR04 conversion divides the measured time by 2, the raw reading of a receiver corresponds to half the total reflected path length: $L_i = 2 \times \text{reading}_i$.

Because every sensor faces upward, no direct transmitter-to-receiver path exists. The shortest significant acoustic path is the ceiling reflection. Later echoes (walls, multipath) are rejected by first-arrival thresholding.

### 2. Ceiling-Reflection Geometry (Virtual Source Method)

Let the moving object be at $(x, y, z)$, where $z$ is measured by the downward-facing VL53L0X, and let the ceiling height be $H = h_c + z$.

By the law of specular reflection, a wave travelling from the object to the ceiling and down to a floor receiver is geometrically equivalent to a straight line from a **virtual mirror source** above the ceiling at height

$$Z_{\text{eff}} = (H - z) + H = 2H - z = 2h_c + z$$

Given the total acoustic path length $L_i$ from the transmitter to the $i$-th floor receiver via the ceiling, the horizontal distance in the $XY$-plane is

$$r_i = \sqrt{L_i^2 - Z_{\text{eff}}^2}$$

![Reflection geometry](images/reflection_geometry.png)

### 3. 2D Trilateration via Nonlinear Least Squares

Once the horizontal distances $r_1, \dots, r_4$ to the receivers at $(x_i, y_i)$ are recovered, the planar coordinates $(x, y)$ are estimated by minimizing the squared range residuals:

$$(\hat{x}, \hat{y}) = \arg\min_{(x, y)} \sum_{i=1}^{4} \left( \sqrt{(x - x_i)^2 + (y - y_i)^2} - r_i \right)^2$$

Because $z$ is already known, three receivers are sufficient for $(x, y)$; the fourth adds redundancy, and a receiver that does not hear the pulse can be dropped. Combining $(\hat{x}, \hat{y})$ with the laser height $z$ yields the full 3D position $(x, y, z)$, which is streamed over serial and visualized live in 3D.

Equivalently, the residual can be written in the path-length domain,
$\sqrt{(x - x_i)^2 + (y - y_i)^2 + Z_{\text{eff}}^2} - L_i$,
which avoids taking the square root of a possibly negative number.

![Trilateration](images/trilateration.png)

---

## Physical prototype

The prototype uses an Arduino Mega 2560 and HC-SR04 modules mounted on a cardboard frame (one transmitter on the moving unit and four corner receivers), with the laser module mounted underneath the moving unit.

![Prototype, top view](images/IMG_7330.jpeg)
![Prototype, side view](images/IMG_7332%20%281%29.jpeg)
![Sensor mounting detail](images/IMG_7335.jpeg)

### Live run

Real-time 3D position display in MATLAB while the hardware is running (ultrasonic channels `d1` to `d5`, laser height `z`, and the estimated `X, Y, Z` coordinates).

![MATLAB live GUI](images/IMG_7331%20%281%29.jpeg)

---

## Repository structure

```text
.
├── arduino/
│   └── ultrasonic_laser_tracker/   # Firmware: 5x HC-SR04 + VL53L0X, CSV over serial
├── matlab/
│   ├── simulation/                 # ToF / IMU / Kalman / complementary comparison
│   └── live_gui/                   # Real-time 3D visualization + trilateration
├── images/                         # Setup photos and result plots
└── README.md
```

## Hardware

| Part | Qty | Role |
|---|---|---|
| HC-SR04 ultrasonic module | 5 (1 transmitter + 4 receivers) | Ceiling clearance + ceiling-reflected time-of-flight ranging |
| VL53L0X / GY-53 laser module | 1 | Direct height ($z$) measurement above floor |
| Arduino Mega 2560 | 1 | Multi-channel echo timing and serial output |

## Getting started

**Firmware**
1. Install the `Adafruit_VL53L0X` library.
2. Open `arduino/ultrasonic_laser_tracker/ultrasonic_laser_tracker.ino` in the Arduino IDE, check the trig/echo pins, and upload to an Arduino Mega 2560.
3. The board prints one CSV line per measurement cycle (5 ultrasonic distances + laser height, in cm) at 9600 baud.

**Live visualization (MATLAB)**
1. Set the serial port in `matlab/live_gui/UltrasonicTrackerApp_Debug.m` (baud rate must match the firmware).
2. Run `UltrasonicTrackerApp_Debug` to see the parsed distances, the estimated $(x, y, z)$, and the 3D scene.

**Simulation (MATLAB)**
Run `matlab/simulation/sensor_fusion_simulation.m`. It requires the Optimization Toolbox (`lsqnonlin`).

---

## Simulation results

The simulation models two measurement sources, ToF-based position and IMU-based motion, and produces the final estimate as a weighted combination of the two (Kalman filter and complementary filter variants).

Setup: 2 × 2 × 2 m cube, four floor sensors, 3D sinusoidal trajectory (10 s, 0.05 s step), Gaussian noise on distance and IMU measurements. Position RMS error in metres, for IMU noise = 0.02:

| ToF noise | ToF only | IMU only (drift) | Kalman | Complementary |
|---|---|---|---|---|
| 0.01 | 0.0187 | 0.0764 | **0.0160** | 0.0410 |
| 0.03 | 0.0651 | 0.1081 | **0.0413** | 0.0458 |
| 0.06 | 0.1157 | 0.1184 | **0.0505** | 0.0518 |

IMU-only integration drifts rapidly; fusing it with ToF measurements via a Kalman filter gives the lowest error at every tested noise level. The complementary filter is a lightweight alternative for low-power microcontrollers. Its fixed weights make it less accurate than ToF alone at very low ToF noise (0.0410 vs 0.0187 at 0.01), but at ToF noise 0.03 and 0.06 it beats ToF alone and comes close to the Kalman filter. Exact values vary slightly between runs due to random noise seeds.

The trajectory and error plots below were generated with IMU noise = 0.05 and ToF noise = 0.03, so they do not correspond to the IMU = 0.02 table above.

![Trajectory comparison](images/trajectory_comparison_imu0.05_tof0.03.png)
![Error over time](images/error_over_time_imu0.05_tof0.03.png)

> **Notes:**
> - The simulation adds noise to direct ToF distances. It does **not** model the ceiling-reflection pipeline (including the $L_i / r_i$ noise amplification described below), so these numbers show the benefit of sensor fusion, not the accuracy of the physical system.
> - IMU fusion (Kalman / complementary filters) is evaluated in simulation only. The physical prototype demonstrates real-time 3D localization using the ultrasonic + laser architecture.

## Error sources and mitigation

- **Hardware:** HC-SR04 timing resolution and noise; laser instability on highly absorptive or angled surfaces.
- **Environmental:** The speed of sound varies by about 0.6 m/s per °C. Ceiling roughness and **wall reflections** can introduce multipath. Since all sensors face upward, no direct path exists. *Mitigation:* temperature compensation and first-arrival thresholding.
- **Noise amplification in the radius:** $\partial r_i / \partial L_i = L_i / r_i$, which is large when the horizontal distance is small compared to $Z_{\text{eff}}$. Errors in $L_i$, $h_c$ and $z$ (the latter two enter through $Z_{\text{eff}} = 2h_c + z$) are amplified accordingly, and the radicand should be clamped at zero when the object is nearly above a receiver.
- **Measurement timing:** If the sensors are read one after another rather than simultaneously, motion of the object between readings adds error. *Mitigation:* short cycle time and low object speed (or motion compensation using the IMU).
- **Geometry (GDOP):** Symmetric receiver placement at the workspace corners minimizes Geometric Dilution of Precision in the trilateration step.
- **Surface assumption:** The model assumes a flat, horizontal reflecting surface above the object; a tilted or uneven ceiling adds error.

## Future work

- Temperature-compensated speed of sound using an onboard thermistor/DHT sensor
- Real-time onboard Kalman filtering integrating a 6-DOF IMU
- Coded acoustic signals (chirp / PRN) to improve SNR and reject multipath
- Experimental measurement of the usable beam angle and the coverage gain over ceiling-mounted receivers
- Quantitative experimental trajectory evaluation against an optical ground-truth reference

## Team and contributions

Team project of three students. Supervisor: Dr. Ayati, University of Tehran, 2025.

- **Adib Saffari**: Overall system architecture and the ceiling-reflection sensing mechanism (one upward transmitter, four floor receivers); geometric formulation for extracting 2D radii from reflected acoustic paths; integration of the downward-facing laser rangefinder for real-time height compensation.
- **Soheil Dehghani** and **Seyed Mohammad Hossein Emami**: IMU-based state estimation (dead-reckoning integration, Kalman and complementary filter design), and MATLAB implementation of the sensor-fusion simulation and numerical solvers.

## License

MIT
