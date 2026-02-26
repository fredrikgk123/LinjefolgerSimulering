# Line-Following Robot Simulation

![Simulation Output](assets/for_readme/demo.gif)

Top-down 2D simulator for the physical ESP32 line-following robot.
Matches the physical robot's sensor array, motor dynamics, and PID logic so that gains tuned here transfer directly to real hardware.

---

## Requirements

```bash
pip install numpy matplotlib pillow
```

---

## Project Structure

```
Simulering/
├── README.md
├── assets/                    ← Track images (PNG, black line on white)
│   ├── bane_fase2.png
│   └── suzuka.png
├── output/                    ← Optimizer results saved here (auto-created)
└── src/
    ├── config.py              ← ★ All parameters live here
    ├── main.py                ← Visual simulation run
    ├── lap_optimizer.py       ← CMA-ES / random / grid lap-time optimizer
    ├── preview_track.py       ← Plot track with spawn + checkpoint zones
    ├── performance_metrics.py ← RMS / settling-time calculations
    ├── control/
    │   └── pid_controller.py  ← PID + SpeedController
    ├── physics/
    │   ├── robot_model.py     ← Differential drive + motor lag + slip
    │   └── friction.py        ← Pacejka tyre friction model
    ├── sensors/
    │   └── qtr_array.py       ← QTRX-HD-25RC array simulation
    ├── track/
    │   └── image_loader.py    ← Track image loader / resizer
    └── visualization/
        └── plots.py           ← Live dashboard + summary plots
```

---

## Quick Start

```bash
cd src

# Visual run (default track: bane_fase2.png)
python3 main.py

# Different track
python3 main.py --track ../assets/suzuka.png

# Preview spawn point and checkpoints
python3 preview_track.py
python3 preview_track.py --track suzuka.png

# Optimize lap time (CMA-ES, 30 generations ~recommended)
python3 lap_optimizer.py
python3 lap_optimizer.py --track suzuka.png --mode cmaes --iterations 50
python3 lap_optimizer.py --mode random --iterations 40
```

---

## How It Works

### PID + error normalisation

The simulator computes a weighted centroid of sensor readings to find the line's lateral position, then normalises it:

```
e_norm = e_y / QTR_HALF_SPAN      # range ±1, same scale as physical readLineBlack ÷ 4000
w_cmd  = PID(e_norm)               # rad/s steering command
```

Physical PID gains translate directly using:

```
SCALE   = 4000 counts × (2 × 2.20/255 / 0.165) = 418.28
Kp_sim  = Kp_phys × SCALE
Kp_phys = Kp_sim  / SCALE
```

### Wheel mix + steering clamp

```python
w_max  = 2 * v_cmd / WHEEL_BASE   # max steering before inner wheel reverses
w_cmd  = clip(w_cmd, -w_max, w_max)
vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2
```

Matches physical robot behaviour — motor driver constrains each wheel PWM to `[0, MAX_PWM]`.

### Speed controller state machine

| State    | Condition | Speed |
|----------|-----------|-------|
| STRAIGHT | `\|e_norm\| < SC_ERROR_THRESHOLD` and low steering | `SC_STRAIGHT_SPEED` |
| TURNING  | otherwise | `SC_TURN_SPEED × max(SC_MIN_SPEED_FACTOR, 1 − turn_factor)` |

Speed transitions are low-pass filtered with coefficient `SC_SMOOTHING`.

### Physics model

- **Motor lag** — first-order lag τ = `MOTOR_TAU` (60 ms)
- **Dead zone** — commands below `MOTOR_DEADZONE` (0.33 m/s) produce no motion
- **Tyre slip** — Pacejka-like curve, reduced normal force from front-heavy CoM
- **Yaw inertia** — angular velocity has its own first-order lag (~50–80 ms heading delay)

### Lap validation

1. Robot departs > `MIN_DEPARTURE_DIST` (0.30 m) from spawn
2. All checkpoints cleared **in order** within `CHECKPOINT_RADIUS` (0.10 m)
3. Robot re-enters `START_FINISH_RADIUS` (0.10 m) after ≥ `MIN_LAP_TIME` (5 s)

If checkpoints are not all cleared, the finish crossing is ignored and the robot keeps running.

---

## Configuration (`config.py`)

All parameters in one file. Key values:

| Parameter | Default | Physical equivalent |
|---|---|---|
| `PID_KP` | 11.7 | Kp_phys = 0.028 |
| `PID_KI` | 0.042 | Ki_phys = 0.0001 |
| `PID_KD` | 50.1 | Kd_phys = 0.1198 |
| `PID_LIMIT` | 23.5 rad/s | MAX_TURN = 225 PWM |
| `PID_DERIV_FILTER` | 0.30 | LP filter α on derivative |
| `SC_STRAIGHT_SPEED` | 0.86 m/s | baseSpeed = 100 PWM |
| `SC_TURN_SPEED` | 0.50 m/s | must be < straight speed |
| `SC_ERROR_THRESHOLD` | 0.25 | normalised error → TURNING mode |
| `SC_SMOOTHING` | 0.08 | speed blend (lower = smoother) |

After an optimizer run, paste the printed block directly into `config.py`.

---

## Optimizer

```bash
python3 lap_optimizer.py --mode cmaes --iterations 30   # recommended
python3 lap_optimizer.py --mode random --iterations 60  # fast baseline
```

Results are saved to `output/lap_<track>_<timestamp>.json`.
The optimizer uses **identical** physics and lap-timer logic to `main.py`.

### Converting sim gains back to physical robot

```
Kp_phys  = PID_KP    / 418.28
Ki_phys  = PID_KI    / 418.28
Kd_phys  = PID_KD    / 418.28
MAX_TURN = round(PID_LIMIT / 0.10457)   # clip to [0, 255]
```

---

## Adding a New Track

1. Drop image into `assets/` (black line on white, any common format)
2. Add spawn in `config.py → SPAWN_REGISTRY`
3. Add checkpoints in `config.py → CHECKPOINT_REGISTRY` (or leave `[]`)
4. Run `python3 preview_track.py --track your_track.png` to verify placement
5. Run `python3 main.py --track ../assets/your_track.png`
