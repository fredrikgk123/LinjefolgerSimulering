# Line-Following Robot Simulator

A simple, self-contained Python simulation of a differential-drive line-following robot with a live matplotlib dashboard. Modelled on the real physical robot hardware listed below.

---

## Hardware

| Component | Part |
|---|---|
| Microcontroller | ESP32 |
| Sensor array | Pololu QTRX-HD-25RC (25 sensors, central 8 wired, 4 mm pitch) |
| Motors | Pololu Micro Metal 6V HP 30:1 |
| Battery | 7.4 V LiPo (2S) |
| Wheels | 34 mm diameter (r = 17 mm), directly on motor shafts |

### Physical measurements

| Parameter | Value | How measured |
|---|---|---|
| Total mass | **270 g** | Scale |
| Wheel base | **180 mm** | Centre-to-centre |
| Axle to sensor bar | **160 mm** | Ruler |
| CoM ahead of axle | **45 mm** | Estimated from layout |
| Chassis height | **< 8 mm** | Ruler (very flat) |
| CoM height | **≈ 4 mm** | Half chassis height |
| Friction coefficient (μ) | **1.12** | Tilt-plane method |
| Wheel contact patch width | **22 mm** | Ruler |
| Sensor bar height off floor | **5 mm** | Rests on screw heads at front |

### Chassis geometry (inverted-T)

The chassis is an inverted-T shape: a wide crossbar at the rear holding the drive wheels (180 mm apart), and a narrow spine extending 160 mm forward to the sensor bar. The robot rests on the two drive wheels at the rear and the sensor-bar screw heads at the front — there is no castor.

```
  sensor bar
  [========]   ← 160 mm from axle, 5 mm off floor
       |
       |  spine (≈40 mm wide)
       |
  [==axle==]   ← drive wheels, 180 mm apart
  L        R
```

### Derived physics values

| Parameter | Calculation | Result |
|---|---|---|
| No-load wheel speed at 7.4 V | 1233 RPM × 2π/60 × 0.017 m | **2.19 m/s** |
| Sensor offset from CoM | 160 mm − 45 mm | **115 mm** |
| Sensor half-span | (8 − 1) / 2 × 4 mm | **14 mm** |
| Max angular rate | 2 × 2.20 / 0.180 m | **24.4 rad/s** |
| Drive wheel normal force at rest | 270g × 9.81 × (115/160) | **1.90 N  (194 g)** |
| Yaw moment of inertia (Iz) | Inverted-T two-slab model | **854 g·m²** |
| Yaw time constant (TAU_YAW) | Iz / (m × (B/2)²) | **390 ms** |
| Weight on drive wheels at rest | 115 / 160 | **71.9 %** |
| Lateral v_max at ω = 8 rad/s | μ × F_normal / (m × ω) | **0.93 m/s** ← grip-limited |
| Target avg speed (8 s lap, ~10 m) | 10 m / 8 s | **1.25 m/s** |

> **Key insight:** The CoM being 45 mm ahead of the axle means the drive wheels carry only ~72 % of the weight at rest, and *less* under hard acceleration. At ω ≥ 8 rad/s (moderate cornering) the lateral grip cap drops below 1.0 m/s — this is the primary performance constraint and why PID tuning is sensitive.

---

## Project Structure

```
Simulering/
├── main.py          # entire simulation — one file, no dependencies beyond pip packages
└── assets/
    ├── bane_fase2.png   # default track
    └── suzuka.png       # alternative track
```

---

## Requirements

- Python 3.9+
- `numpy`
- `matplotlib`
- `Pillow`

Install with:

```bash
pip install numpy matplotlib Pillow
```

---

## Running the Simulation

```bash
# Default track (bane_fase2.png)
python main.py

# Alternative track
python main.py --track assets/suzuka.png
```

Close the window or press `Ctrl-C` to stop.

---

## How It Works

### Simulation Loop
Each timestep (`DT = 0.005 s`, 200 Hz — matches the ESP32 RC-sensor read rate) the simulator:
1. Reads the 25-channel sensor array
2. Estimates lateral error from a weighted average of sensor positions
3. Feeds the normalised error (±1 = robot at edge of array) into a PID controller → angular velocity command
4. A hysteresis speed controller slows the robot in corners
5. Updates robot physics (first-order motor lag, dead zone, differential-drive kinematics)
6. Renders the live dashboard every 4 steps

### Components (all in `main.py`)

| Component | Description |
|---|---|
| **Track loader** | Opens any grayscale PNG, auto-inverts dark backgrounds, resizes to simulation resolution |
| **Sensor array** | 8 active channels (central 8 of 25) on a QTRX-HD-25RC, 4 mm pitch, mounted 97 mm ahead of centre. Includes Gaussian noise and 10-bit quantisation |
| **PID controller** | Normalised error (±1 = edge of sensor array), anti-windup integral clamp, low-pass filtered derivative |
| **Speed controller** | Hysteresis state machine — full speed on straights, scaled-down speed in corners |
| **Robot physics** | First-order motor lag (τ = 55 ms, HP 30:1 loaded), dead zone, differential-drive kinematics |
| **Dashboard** | Live matplotlib window — track view, 25-bar sensor chart, lateral error history, wheel speed history |

---

## Tuning Parameters

All parameters are at the top of `main.py` under clearly labelled sections:

| Section | Key parameters |
|---|---|
| **PID** | `KP`, `KI`, `KD`, `PID_LIMIT`, `INTEG_LIMIT`, `DERIV_ALPHA` |
| **Speed** | `STRAIGHT_SPD`, `TURN_SPD`, `ERR_THRESH`, `MIN_SPD_F`, `ACCEL_RATE` |
| **Hardware** | `WHEEL_BASE`, `WHEEL_RADIUS`, `MAX_WHEEL`, `MOTOR_TAU`, `DEADZONE` |
| **Sensor** | `N_SENSORS`, `SPACING_M`, `SENSOR_FWD`, `NOISE_STD` |
| **Spawn** | `SPAWNS` dict — starting position and heading for each track |

### Tuning for ~8 s lap times
The values below are calibrated for the physical hardware at 7.4 V. The straight speed (1.60 m/s) is ~73 % of the no-load maximum, giving a comfortable safety margin while still achieving the ~1.25 m/s average needed for an 8-second lap.

```
STRAIGHT_SPD = 1.60    # ramp to this on straights (m/s)
TURN_SPD     = 0.85    # base speed through corners (m/s)
KP           = 85.0    # proportional — main steering authority
KI           =  0.8    # integral — corrects heading drift
KD           =  4.5    # derivative — damps oscillation
```

---

## Dashboard

| Panel | Shows |
|---|---|
| **Robot Live View** (left) | Track map with robot position, heading arrow, 25 sensor dots, and driven path |
| **Sensor readings** (top right) | Bar chart of all 8 active QTRX-HD-25RC channel values (green = on line) |
| **Lateral error** (middle right) | Rolling 5-second history of lateral error in metres |
| **Wheel speeds** (bottom right) | Rolling 5-second history of left/right wheel speeds |

