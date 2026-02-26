# Line-Following Robot Simulation

![Simulation Output](assets/for_readme/demo.gif)
*Live simulation view showing the robot tracking a circuit, with real-time sensor readings, lateral error, and wheel speed plots. Yellow checkpoint circles turn green as the robot clears them.*

---

## Requirements

```bash
pip install numpy matplotlib pillow
```

---

## Structure

```
Simulering/
├── src/
│   ├── main.py                    ← Entry point (visual run)
│   ├── config.py                  ← ★ Single source of truth for ALL parameters
│   ├── lap_optimizer.py           ← Lap-time optimizer (CMA-ES / random / grid)
│   ├── performance_metrics.py     ← RMS / settling time calculations
│   ├── optimize.py                ← General PID optimizer CLI
│   ├── preview_track.py           ← Plot track with start/finish zone for inspection
│   ├── pid_optimizer.py           ← ML optimization engine (grid / bayesian)
│   ├── multi_track_simulator.py   ← Run simulations across multiple tracks
│   ├── multi_track_plots.py       ← Stacked list / bar / table visualizations
│   ├── control/
│   │   └── pid_controller.py      ← PID + SpeedController state machine
│   ├── physics/
│   │   ├── robot_model.py         ← Differential drive kinematics
│   │   └── friction.py            ← Tyre friction model
│   ├── sensors/
│   │   └── qtr_array.py           ← QTR-HD-25RC sensor array simulation
│   ├── track/
│   │   ├── track_generator.py     ← Default sine-wave track (generated)
│   │   ├── image_loader.py        ← Universal loader for custom track images
│   │   └── multi_track.py         ← Built-in tracks for optimizer testing
│   ├── visualization/
│   │   ├── plots.py               ← Live + summary plots (incl. checkpoint circles)
│   │   └── sensor_overlay.py      ← Sensor reading overlay
│   └── tests/
│       ├── debug_sensors.py
│       ├── test_multi_track.py
│       ├── test_sensor.py
│       ├── test_sensor_tracking.py
│       └── test_setup.py
├── assets/                        ← Track images (add your own here)
│   ├── suzuka.png
│   ├── bane_fase2.png
│   └── for_readme/
├── docs/                          ← Detailed documentation
├── output/                        ← Generated results (auto-created)
└── README.md
```

---

## Run Simulation

```bash
cd src

python3 main.py                                   # default sine-wave track
python3 main.py --track ../assets/bane_fase2.png  # Competition track
python3 main.py --track ../assets/suzuka.png      # Suzuka circuit
python3 main.py --track ../assets/my_track.png    # Any image you add
```

> **Adding your own track:** drop any top-down PNG/JPG into `assets/` and pass it with `--track`.  
> Image requirements: **dark line on a light background, top-down view**.

---

## Configuration — one file to rule them all

**Everything you would ever want to tune lives in `src/config.py`.**  
Both the visual run (`main.py`) and the lap-time optimizer (`lap_optimizer.py`) import from it,
so changing a value there instantly affects both programs — no need to edit two files.

### PID + Speed Controller

```python
# src/config.py

PID_KP               = 120.0   # proportional gain
PID_KI               = 4.0     # integral gain
PID_KD               = 18.0    # derivative gain
PID_LIMIT            = 22.0    # max angular-velocity command (rad/s)
PID_INTEGRAL_LIMIT   = 1.2     # anti-windup clamp
PID_DERIV_FILTER     = 0.10    # low-pass on derivative term

SC_STRAIGHT_SPEED    = 0.91    # m/s on straights
SC_TURN_SPEED        = 0.70    # m/s in corners
SC_ERROR_THRESHOLD   = 0.007   # lateral error (m) that triggers speed reduction
SC_SMOOTHING         = 0.12    # first-order blend between speed states
```

The optimizer uses these values as its **initial guess** — so if you paste a new best result
from the optimizer output back here, the next run starts from that better point.

### Spawn positions

```python
SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 1.00, "y": 0.14, "theta":  0.00},
    "suzuka.png":     {"x": 0.55, "y": 0.68, "theta": -0.40},
    "my_track.png":   {"x": 1.50, "y": 0.50, "theta":  0.00},  # ← add yours
}
```

`x`/`y` are metres from the bottom-left corner, `theta` in radians (`0` = right, `π/2` = up).  
This one entry is used automatically by `main.py`, `lap_optimizer.py` and `multi_track_simulator.py`.

### Lap-timer settings

```python
START_FINISH_RADIUS = 0.10   # metres — robot must re-enter this circle to finish
MIN_DEPARTURE_DIST  = 0.30   # metres — must leave start zone before finish counts
MAX_LAP_TIME        = 60.0   # seconds — DNF cutoff in optimizer
MAX_LINE_LOSS_TIME  = 1.0    # seconds of continuous line loss before DNF
```

### Simulation settings

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SIM_TIME` | 45.0 s | Visual-run duration cap |
| `DT` | 0.005 s | Physics timestep |
| `MAP_SIZE_M` | (4.0, 2.0) | Track area in metres |
| `TRACK_WIDTH_M` | 0.020 m | Line width |
| `QTR_CHANNELS` | 25 | Sensor channels |
| `MAX_WHEEL_SPEED` | 1.0 m/s | Motor speed limit |
| `WHEEL_BASE` | 0.12 m | Distance between wheels |
| `NOISE_SEED` | 42 | Fixed seed → deterministic runs. Set `None` for random noise. |

---

## Checkpoint System

Checkpoints prevent the optimizer (and the visual run) from recording a valid lap if the robot
cuts across the track or teleports to the finish line. The robot must pass through **every
checkpoint in order** before the finish line is counted.

### Visual cues

In the live view (`main.py`) each checkpoint is drawn as a **circle** matching `CHECKPOINT_RADIUS`:

| Colour | Meaning |
|--------|---------|
| 🟡 Yellow | Checkpoint not yet reached |
| 🟢 Green | Checkpoint cleared |

The circle label shows the checkpoint number. In the terminal you will see:
```
Checkpoint system active: 4 checkpoints must be cleared in order.
  Checkpoint 1/4 cleared at t=3.21s
  Checkpoint 2/4 cleared at t=7.84s
  ...
*** LAP COMPLETE — time: 14.237 s ***
```

If the robot reaches the finish line without clearing all checkpoints:
```
*** FALSE LAP — only 2/4 checkpoints cleared. Continuing... ***
```

### Adding / tuning checkpoints

Checkpoints are defined in `src/config.py`:

```python
CHECKPOINT_RADIUS = 0.18   # metres — how close the robot must get to clear a checkpoint

CHECKPOINT_REGISTRY = {
    "bane_fase2.png": [
        (2.80, 0.14),   # 25 % — far end of bottom straight
        (3.50, 1.00),   # 50 % — right-hand hairpin apex
        (2.00, 1.85),   # 75 % — top straight midpoint
        (0.40, 1.00),   # 100% — left-hand hairpin apex
    ],
    "suzuka.png": [
        (1.80, 0.68),
        (2.80, 1.20),
        (1.50, 1.60),
        (0.55, 1.20),
    ],
    # "my_track.png": [(x1,y1), (x2,y2), ...],
}
```

**Tip — finding good positions:**  
Run `main.py` once and watch the robot's path on the live map. Pick four evenly-spaced points
around the circuit (roughly 25 %, 50 %, 75 % and 100 % of the lap). The yellow circles are
drawn at those coordinates so you can immediately see if they sit on the track.  
Increase `CHECKPOINT_RADIUS` if a checkpoint is never cleared on a valid run.

To **disable checkpoints** for a track, set an empty list:
```python
"my_track.png": [],
```

---

## Run Lap-Time Optimizer

Optimizes PID + SpeedController to **minimize lap time** using CMA-ES by default.  
The optimizer starts from the values defined in `config.py` and the checkpoint system
runs inside every evaluation, so cheating routes are automatically rejected.

```bash
cd src

python3 lap_optimizer.py                                           # CMA-ES, bane_fase2, 30 gen
python3 lap_optimizer.py --track bane_fase2.png --iterations 60   # more generations
python3 lap_optimizer.py --track suzuka.png                        # different track
python3 lap_optimizer.py --mode random --iterations 50             # fast random baseline
python3 lap_optimizer.py --mode grid                               # exhaustive (slow)
```

| Mode | Algorithm | Notes |
|------|-----------|-------|
| `cmaes` | CMA-ES evolution strategy | **Recommended** — adapts search direction automatically |
| `random` | Uniform random sampling | Fast baseline |
| `grid` | Exhaustive grid | Very slow — use only for small spaces |

After a run, the best parameters are printed as **ready-to-paste code** and saved to
`output/lap_<track>_<timestamp>.json`. Paste the values into `config.py` so that the next
optimizer run (and `main.py`) both start from the improved baseline.

---

## Run General PID Optimizer

Optimizes PID + SpeedController across all tracks in the registry (`sine`, `bane_fase2`, `suzuka`).

```bash
cd src

python3 optimize.py --mode quick                      # ~13 min  | 27 combinations  | recommended
python3 optimize.py --mode bayesian                   # ~75 min  | 30 random samples | refinement
python3 optimize.py --mode bayesian --iterations 100  # ~4 hrs   | more samples
python3 optimize.py --mode full                       # 50+ hrs  | 6400 combinations | exhaustive
```

Results saved to `output/optimization_results.json`.

---

## Run Multi-Track Test

```bash
cd src/tests
python3 test_multi_track.py
```

Runs the current PID on all tracks in the registry and saves comparison plots to `output/`:

| Plot | Description |
|------|-------------|
| `multi_track_results.png` | Stacked list with EXCELLENT / GOOD / OK / POOR rating per track |
| `multi_track_bars.png` | Side-by-side bar charts (Max error, RMS, Settling time) |
| `multi_track_table.png` | Summary table with aggregated averages |

> **Adding a track to the registry:** add a line to `ALL_TRACKS` in `src/track/multi_track.py`:
> ```python
> "my_track": lambda: _load_asset("my_track.png"),
> ```

---

## Workflow — typical tuning loop

```
1. Run main.py → watch the live view, adjust checkpoint positions in config.py
                  until all circles sit on the track and turn green on a clean lap.

2. Run lap_optimizer.py → let CMA-ES run for 30–60 generations.

3. Copy the "BEST" parameters from the console output into config.py
   (PID_KP, PID_KI, … SC_STRAIGHT_SPEED, …).

4. Run main.py again → visually verify the lap looks correct and all
   checkpoints go green in the right order.

5. Repeat from step 2 to refine further.
```
