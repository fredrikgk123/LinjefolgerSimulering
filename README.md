# Line-Following Robot Simulation

![Simulation Output](assets/for_readme/demo.gif)
*Live simulation view showing the robot tracking a circuit, with real-time sensor readings, lateral error, and wheel speed plots. Yellow checkpoint circles turn green as the robot clears them.*

---

## Requirements

```bash
pip install numpy matplotlib pillow
```

---

## Project Structure

```
Simulering/
├── src/
│   ├── config.py              ← ★ ALL parameters live here — start here
│   ├── main.py                ← Entry point (visual run)
│   ├── lap_optimizer.py       ← Lap-time optimizer (CMA-ES / random / grid)
│   ├── performance_metrics.py ← RMS / settling time calculations
│   ├── preview_track.py       ← Plot track with start/finish zone for inspection
│   ├── control/
│   │   └── pid_controller.py  ← PID + SpeedController state machine
│   ├── physics/
│   │   ├── robot_model.py     ← Differential drive kinematics
│   │   └── friction.py        ← Tyre friction model
│   ├── sensors/
│   │   └── qtr_array.py       ← QTR-HD-25RC sensor array simulation
│   ├── track/
│   │   └── image_loader.py    ← Universal loader for track images
│   ├── visualization/
│   │   └── plots.py           ← Live + summary plots (incl. checkpoint circles)
│   └── tests/
│       ├── debug_sensors.py   ← Sensor debug at spawn position
│       └── test_setup.py      ← Import + simulation sanity check
├── assets/                    ← Track images
│   ├── bane_fase2.png
│   ├── suzuka.png
│   └── for_readme/
├── docs/                      ← Detailed documentation
├── output/                    ← Generated results (auto-created)
└── README.md
```

---

## Quick Start

```bash
cd src

python3 main.py --track ../assets/bane_fase2.png  # Competition track
python3 main.py --track ../assets/suzuka.png      # Suzuka circuit
python3 main.py --track ../assets/my_track.png    # Any image you add
```

> **Adding your own track:** drop any top-down PNG/JPG into `assets/` and pass it with `--track`.  
> Image requirements: **dark line on a light background, top-down view**.  
> Add a spawn position and checkpoints for it in `config.py` (see §SPAWN and §CHECKPOINTS below).

---

## Configuration — `src/config.py`

**Every tuneable value lives in `src/config.py`.**  
Both `main.py` and `lap_optimizer.py` import from it — change a value once and both programs pick it up immediately.

The file is divided into clearly labelled sections with a **quick-find index** at the top.  
Search for the section tag (e.g. `§PID`) to jump straight to what you need.

| # | Section tag | What you can change |
|---|-------------|---------------------|
| 1 | `§PID` | KP / KI / KD gains, output clamp, anti-windup, derivative filter |
| 2 | `§SPEED` | Straight speed, turn speed, error threshold, smoothing |
| 3 | `§HARDWARE` | Wheel base, max wheel speed, motor time-constant, mass, friction |
| 4 | `§SENSOR` | Channel count, pitch, mounting offset, noise, ADC bits, random seed |
| 5 | `§SPAWN` | Starting x / y / heading for each track image |
| 6 | `§CHECKPOINTS` | Checkpoint coordinates and radius per track |
| 7 | `§LAP` | Start/finish radius, departure distance, DNF time limits |
| 8 | `§SIM` | Physics timestep, simulation duration cap, map resolution |

---

### §PID — PID Controller

Controls how the robot steers to stay on the line.  
The optimizer uses these as its **initial guess** — paste better values from the optimizer output here to build on previous results.

```python
# src/config.py  →  §PID

PID_KP             = 120.8   # proportional gain — aggressiveness of steering response
PID_KI             = 3.94    # integral gain     — corrects persistent steady-state error
PID_KD             = 17.1    # derivative gain   — damps oscillation
PID_LIMIT          = 21.6    # max angular-velocity command (rad/s)
PID_INTEGRAL_LIMIT = 1.2     # anti-windup clamp on the integral accumulator
PID_DERIV_FILTER   = 0.10    # low-pass coefficient on derivative term (0–1)
```

---

### §SPEED — Speed Controller

Slows the robot in corners and speeds it up on straights.

```python
# src/config.py  →  §SPEED

SC_STRAIGHT_SPEED  = 0.753   # m/s — target speed on straights
SC_TURN_SPEED      = 0.499   # m/s — target speed in corners
SC_ERROR_THRESHOLD = 0.0250  # m   — lateral error above which "corner mode" kicks in
SC_SMOOTHING       = 0.093   # blending coefficient (lower = snappier transitions)
```

---

### §HARDWARE — Robot Hardware / Physical Properties

Match these to your real robot's physical specifications.

```python
# src/config.py  →  §HARDWARE

WHEEL_BASE        = 0.12   # metres — distance between the two drive wheels
MAX_WHEEL_SPEED   = 1.0    # m/s   — motor hardware speed limit
MOTOR_TAU         = 0.05   # s     — motor time-constant (50 ms)
ROBOT_MASS        = 0.9    # kg
MU_SLIDE          = 1.14   # tyre sliding friction coefficient
MAX_LATERAL_ERROR = 0.05   # m     — error threshold for "off-track" detection
```

---

### §SENSOR — QTR-HD-25RC Sensor Array

Simulation of the Pololu QTR-HD-25RC reflectance sensor.

```python
# src/config.py  →  §SENSOR

QTR_CHANNELS        = 25      # number of sensor channels
QTR_SPACING_M       = 0.004   # m — 4 mm pitch; total array span ≈ ±48 mm
QTR_SENSOR_OFFSET_M = 0.03    # m — sensor mounted 3 cm ahead of centre of mass
QTR_NOISE_STD       = 0.01    # Gaussian noise standard deviation on readings
QTR_ADC_BITS        = 12      # ADC resolution (12-bit = 0–4095 counts)
NOISE_SEED          = 42      # fixed seed → deterministic runs; set None for random
```

---

### §SPAWN — Spawn Positions Per Track

The robot's starting position and heading for each track.

```python
# src/config.py  →  §SPAWN

SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 2.00, "y": 0.14, "theta":  0.00},
    "suzuka.png":     {"x": 0.55, "y": 0.68, "theta": -0.40},
    # "my_track.png": {"x": 1.00, "y": 0.50, "theta":  0.00},  # ← add yours here
}
```

`x`/`y` are metres from the **bottom-left** corner of the image.  
`theta` is in radians (`0` = pointing right, `π/2 ≈ 1.57` = pointing up).

---

### §CHECKPOINTS — Checkpoint Positions Per Track

Checkpoints ensure the robot completes the full circuit in order (no shortcuts).  
The robot must pass within `CHECKPOINT_RADIUS` metres of each point, in order, before a lap is counted.

```python
# src/config.py  →  §CHECKPOINTS

CHECKPOINT_RADIUS = 0.10   # metres — how close the robot must get to clear a checkpoint

CHECKPOINT_REGISTRY = {
    "bane_fase2.png": [
        (2.80, 0.14),   # 25 % — far end of bottom straight
        (3.80, 1.70),   # 50 % — top right corner
        (2.00, 0.50),   # 75 % — bottom of box
        (0.20, 1.70),   # 100% — top left corner
    ],
    "suzuka.png": [
        (1.87, 0.68),   # 25 % — after the first chicane
        (3.87, 1.30),   # 50 % — mid-sector 2
        (1.95, 1.45),   # 75 % — top of the circuit
        (0.15, 1.05),   # 100% — return section
    ],
    # "my_track.png": [(x1, y1), (x2, y2), (x3, y3), (x4, y4)],
}
```

**Finding good checkpoint positions:**  
Run `main.py` and watch the live map. Pick four evenly-spaced waypoints around the circuit.  
Yellow circles show each checkpoint — adjust coordinates until they sit on the track line.  
Increase `CHECKPOINT_RADIUS` if a checkpoint is never cleared on a valid run.  
Set a track's list to `[]` to disable checkpoints for it entirely.

**Visual cues in the live view:**

| Colour | Meaning |
|--------|---------|
| 🟡 Yellow | Checkpoint not yet reached |
| 🟢 Green | Checkpoint cleared |

---

### §LAP — Lap-Timer / DNF Rules

```python
# src/config.py  →  §LAP

START_FINISH_RADIUS = 0.10   # m   — robot must re-enter this circle to finish a lap
MIN_DEPARTURE_DIST  = 0.30   # m   — must leave start zone before finish counts
MAX_LAP_TIME        = 60.0   # s   — any lap slower than this = DNF in optimizer
MAX_LINE_LOSS_TIME  = 0.3    # s   — continuous line-loss longer than this = DNF
```

---

### §SIM — Simulation Settings

Low-level settings you rarely need to touch.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DT` | `0.005` s | Physics timestep (200 Hz) |
| `SIM_TIME` | `45.0` s | Hard cap on visual-run duration |
| `PX_PER_METER` | `500` | Image resolution scale |
| `MAP_SIZE_M` | `(4.0, 2.0)` | Track area in metres (width × height) |

---

## Checkpoint System

If the robot reaches the finish line without clearing all checkpoints:
```
*** FALSE LAP — only 2/4 checkpoints cleared. Continuing... ***
```

When all checkpoints are cleared in order:
```
  Checkpoint 1/4 cleared at t=3.21s
  Checkpoint 2/4 cleared at t=7.84s
  ...
*** LAP COMPLETE — time: 14.237 s ***
```

---

## Preview Track

Plots the track image with spawn point, start/finish zone, departure circle and checkpoint circles overlaid.

```bash
cd src

python3 preview_track.py --track ../assets/bane_fase2.png  # default track
python3 preview_track.py --track ../assets/suzuka.png
```

Output is also saved to `output/track_preview.png`.

---

## Lap-Time Optimizer

Optimizes PID + Speed Controller to **minimize lap time** using CMA-ES.  
Starts from the values in `config.py` (§PID and §SPEED) and respects the checkpoint system.

```bash
cd src

python3 lap_optimizer.py                                           # CMA-ES, bane_fase2, 30 gen
python3 lap_optimizer.py --track ../assets/bane_fase2.png --iterations 60
python3 lap_optimizer.py --track ../assets/suzuka.png
python3 lap_optimizer.py --mode random --iterations 50             # fast random baseline
python3 lap_optimizer.py --mode grid                               # exhaustive (slow)
```

| Mode | Algorithm | Notes |
|------|-----------|-------|
| `cmaes` | CMA-ES evolution strategy | **Recommended** — adapts search direction automatically |
| `random` | Uniform random sampling | Fast baseline |
| `grid` | Exhaustive grid | Very slow — use only for small parameter spaces |

After a run, the best parameters are printed as **ready-to-paste code** and saved to  
`output/lap_<track>_<timestamp>.json`. Paste them into `config.py` (§PID / §SPEED) so the next run starts from the improved baseline.

---

## Typical Tuning Workflow

```
1. Run preview_track.py
     → Verify spawn point and checkpoint circles sit on the track.
     → Adjust §SPAWN and §CHECKPOINTS in config.py if needed.

2. Run main.py
     → Watch the live view and confirm all checkpoints turn green on a clean lap.

3. Run lap_optimizer.py (30–60 generations)
     → CMA-ES searches for faster PID + speed settings.

4. Copy the "BEST" block from the console into config.py (§PID and §SPEED).

5. Run main.py again
     → Visually verify the lap is clean and all checkpoints go green in order.

6. Repeat from step 3 to refine further.
```
