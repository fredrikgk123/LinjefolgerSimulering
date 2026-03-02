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
├── main.py        # entry point — CLI argument parsing only
├── config.py      # all tunable parameters  ← edit this to tune the robot
├── physics.py     # hardware constants derived from measurements  ← do not edit
├── simulator.py   # all backend logic (FSM, physics, dashboard, tuner)
└── assets/
    ├── bane_fase2.png     # default track
    ├── suzuka.png         # alternative track
    └── tuner_best.txt     # written automatically after each tuner run
```

| File | Purpose | Edit? |
|---|---|---|
| `config.py` | PID gains, FSM thresholds, speeds, search space, spawn points, checkpoints | ✅ Yes |
| `physics.py` | Motor constants, mass, wheel geometry, friction, inertia, sensor geometry | ❌ No |
| `simulator.py` | `Robot`, `FSM`, sensor model, dashboard, episode runner, headless, tuner | Only to change behaviour |
| `main.py` | CLI flags, routes to the right function | Only to add new flags |

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

### GUI mode *(default)*
Opens a live matplotlib dashboard with the track view, sensor bars, error history, and wheel speeds.

```bash
# Default track
python main.py

# Alternative track
python main.py --track assets/suzuka.png
```

Close the window or press `Ctrl-C` to stop.

---

### Headless mode *(fast, console only)*
No window — runs at ~50× realtime. A progress bar updates in-place every simulated second; checkpoint and lap events print above it.

```bash
# Run until MAX_TIME (80 s default)
python main.py --headless
python main.py -H                              # short flag

# Stop after N valid laps
python main.py -H --laps 3
python main.py -H -n 3                        # short flag

# Different track, stop after 1 lap
python main.py -H --track assets/suzuka.png -n 1
```

---

### Tuner mode *(Uniform Random Search)*
Randomly samples the PID / FSM parameter space, scores each candidate on lap time and consistency, then prints a ranked table with ready-to-paste values.

```bash
# Default: 500 samples, 60 s sim budget each, show top 5
python main.py --tune
python main.py -T                              # short flag

# Custom sample count and time budget
python main.py --tune --samples 200 --tune-time 40

# More results, fixed seed for reproducibility
python main.py --tune --samples 1000 --top-k 10 --seed 42

# Tune on a different track
python main.py --tune --track assets/suzuka.png --samples 300
```

The best result is also written to `assets/tuner_best.txt` automatically.

---

### All flags at a glance

| Flag | Short | Default | Description |
|---|---|---|---|
| `--track PATH` | | `assets/bane_fase2.png` | Track image to use |
| `--headless` | `-H` | off | Console-only run, no GUI |
| `--laps N` | `-n N` | `0` (no limit) | Stop after N valid laps (headless only) |
| `--tune` | `-T` | off | Run Uniform Random Search tuner |
| `--samples N` | | `500` | Number of random candidates (tuner) |
| `--tune-time S` | | `60.0` | Simulated seconds per candidate (tuner) |
| `--top-k N` | | `5` | Results to display (tuner) |
| `--seed N` | | `0` | RNG seed for reproducibility (tuner) |

---

## How It Works

### Simulation Loop
Each timestep (`DT = 0.005 s`, 200 Hz — matches the ESP32 RC-sensor read rate) the simulator:
1. Reads the 8 active sensor channels
2. Estimates lateral error from a weighted centroid of sensor positions
3. Feeds the normalised error (±1 = robot at edge of array) into a PID controller → angular velocity command
4. A Finite State Machine (FSM) selects the PID gain set and target speed based on how far off-centre the robot is
5. Updates robot physics (back-EMF motor model, yaw inertia, weight transfer, lateral grip cap)
6. Renders the live dashboard every 4 steps (GUI mode only)

### FSM States

| State | Trigger | Colour |
|---|---|---|
| **STRAIGHT** | `\|e\|` < `FSM_STR_EXIT` | Green |
| **CORNER** | `FSM_STR_EXIT` ≤ `\|e\|` < `FSM_SHARP_ENTER` | Orange |
| **SHARP** | `\|e\|` ≥ `FSM_SHARP_ENTER` | Red |
| **LOST** | Sensor weight below threshold | Grey |

### Lap Validation
Checkpoints are placed around the track. A lap is only counted if **all checkpoints are hit in order** before crossing the start/finish line. Hitting a checkpoint out of order immediately voids the lap.

### Components

| Component | File | Description |
|---|---|---|
| **Track loader** | `simulator.py` | Opens any grayscale PNG, auto-inverts dark backgrounds, resizes to simulation resolution |
| **Sensor array** | `simulator.py` | 8 active channels (central 8 of 25) on a QTRX-HD-25RC, 4 mm pitch, 115 mm ahead of CoM. Includes Gaussian noise and 10-bit quantisation |
| **FSM + PID** | `simulator.py` | Four gain sets (STRAIGHT / CORNER / SHARP / LOST), normalised error, filtered derivative, anti-windup integral clamp |
| **Robot physics** | `simulator.py` | Back-EMF motor model, yaw moment of inertia, longitudinal weight transfer, lateral grip cap |
| **Dashboard** | `simulator.py` | Live matplotlib window — track map, sensor bars, lateral error history, wheel speed history, checkpoint markers |
| **Headless runner** | `simulator.py` | Physics-identical loop with no matplotlib; in-place progress bar, ~50× realtime |
| **URS Tuner** | `simulator.py` | Uniform Random Search over 19 PID/FSM parameters; scores by best lap time + consistency |
| **Tunable config** | `config.py` | All PID gains, FSM thresholds, speeds, search space bounds, track layout |
| **Hardware constants** | `physics.py` | Motor, wheel, sensor, friction, inertia — derived from physical measurements |

---

## Tuning Parameters

All tunable parameters are in `config.py`. Physical hardware values live in `physics.py` and should not be changed unless the hardware changes.

| Group | Parameter(s) in `config.py` | Description |
|---|---|---|
| **PID per state** | `PID_STRAIGHT`, `PID_CORNER`, `PID_SHARP` — each `(kp, ki, kd, limit, speed)` | Gains and target speed for each FSM state |
| **Derivative filter** | `DERIV_ALPHA` | EMA weight on the derivative (0 = frozen, 1 = raw) |
| **FSM thresholds** | `FSM_STR_EXIT/ENTER`, `FSM_SHARP_ENTER/EXIT` | Normalised error boundaries for state transitions |
| **Lost search** | `FSM_LOST_SEARCH_W` | Yaw rate while scanning for the line (rad/s) |
| **Acceleration** | `ACCEL_RATE` | Speed ramp rate (m/s²) |
| **Tuner search space** | `SEARCH_SPACE` | `(low, high)` range for each tunable scalar — narrow around a known-good value for a fine search |
