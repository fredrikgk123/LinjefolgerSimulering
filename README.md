# Line-Following Robot Simulation

![Simulation Output](assets/for_readme/demo.gif)
*Live simulation view (left) showing the robot tracking a sine wave track, with real-time sensor array readings, lateral error, and wheel speed plots (right).*

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
│   ├── main.py                    ← Entry point
│   ├── config.py                  ← Simulation parameters (speed, DT, map size…)
│   ├── performance_metrics.py     ← RMS / settling time calculations
│   ├── optimize.py                ← General PID optimizer CLI
│   ├── lap_optimizer.py           ← Lap-time optimizer for bane_fase2
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
│   │   ├── plots.py               ← Live + summary plots
│   │   └── sensor_overlay.py      ← Sensor reading overlay
│   └── tests/
│       ├── debug_sensors.py       ← Sensor position debug tool
│       ├── test_multi_track.py    ← Multi-track evaluation test
│       ├── test_sensor.py
│       ├── test_sensor_tracking.py
│       └── test_setup.py
├── assets/                        ← Track images (add your own here)
│   ├── suzuka.png
│   ├── bane_fase2.png
│   └── for_readme/
│       ├── image1.png
│       └── demo.gif
├── docs/                          ← Detailed documentation
├── output/                        ← Generated plots and results (auto-created)
└── README.md
```

---

## Run Simulation

```bash
cd src

python3 main.py                                   # default sine-wave track
python3 main.py --track ../assets/suzuka.png      # Suzuka circuit
python3 main.py --track ../assets/bane_fase2.png  # Competition track
python3 main.py --track ../assets/my_track.png    # Any image you add
```

> **Adding your own track:** drop any top-down PNG/JPG into `assets/` and pass it with `--track`.  
> The robot spawns at the centre and searches for the line automatically.  
> Image requirements: **dark line on a light background, top-down view**.

**Hardcoding a spawn point** for a known track — edit `src/config.py`:
```python
SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 2.00, "y": 0.14, "theta":  0.00},
    "suzuka.png":     {"x": 0.55, "y": 0.65, "theta": -0.80},
    "my_track.png":   {"x": 0.20, "y": 0.80, "theta":  1.57},  # <- add yours
}
```
`x`/`y` are in metres (world coordinates, origin = bottom-left), `theta` in radians (`0` = pointing right, `π/2` = pointing up).

---

## Run Lap-Time Optimizer (bane_fase2)

Optimizes PID + SpeedController to **minimize lap time** on any track. Uses CMA-ES (Covariance Matrix Adaptation Evolution Strategy) by default — an evolutionary algorithm that adapts its search direction automatically, much more efficient than grid or random search.

```bash
cd src

# CMA-ES (recommended) #30 iterations, default = ~15 minutes
python3 lap_optimizer.py --track bane_fase2.png --iterations 60   # more generations
python3 lap_optimizer.py --track suzuka.png                        # different track

# Random search — fast baseline
python3 lap_optimizer.py --mode random --iterations 50

# Grid — exhaustive (very slow)
python3 lap_optimizer.py --mode grid
```

| Mode | Algorithm | Notes |
|------|-----------|-------|
| `cmaes` | CMA-ES evolution strategy | **Recommended** — adapts to problem geometry |
| `random` | Uniform random sampling | Fast baseline |
| `grid` | Exhaustive grid | Very slow, use only for small spaces |

| Setting | Value | Description |
|---------|-------|-------------|
| Start/finish zone | 10 cm radius | Robot must re-enter this after leaving |
| Invalid if | line lost > 1.0 s | Continuous line loss = off track = DNF |
| Max lap time | 60 s | DNF if not completed within this |

Results saved to `output/lap_<track>_<timestamp>.json`. Best parameters printed as ready-to-paste code.

**Adding a new track:** add a spawn entry to `SPAWN_REGISTRY` in `src/config.py`:
```python
SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 2.00, "y": 0.14, "theta": 0.0},
    "my_track.png":   {"x": 1.50, "y": 0.50, "theta": 0.0},  # <- add yours
}
```
This one entry is used automatically by `main.py`, `lap_optimizer.py` and `multi_track_simulator.py`.

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

Results saved to `output/optimization_results.json`. Best parameters printed as ready-to-paste code.

---

## Run Multi-Track Test

```bash
cd src/tests
python3 test_multi_track.py
```

Runs your current PID on all tracks in the registry (`sine`, `bane_fase2`, `suzuka`) and saves comparison plots to `output/`:

> **Adding a track to the registry:** add a line to `ALL_TRACKS` in `src/track/multi_track.py`:
> ```python
> "my_track": lambda: _load_asset("my_track.png"),
> ```

| Plot | Description |
|------|-------------|
| `multi_track_results.png` | Stacked list with EXCELLENT / GOOD / OK / POOR rating per track |
| `multi_track_bars.png` | Side-by-side bar charts (Max error, RMS, Settling time) |
| `multi_track_table.png` | Summary table with aggregated averages |

---

## Configuration

All key parameters live in `src/config.py`:

| Parameter | Default    | Description |
|-----------|------------|-------------|
| `SIM_TIME` | 30.0 s      | Simulation duration |
| `DT` | 0.005 s    | Physics timestep |
| `MAP_SIZE_M` | (4.0, 2.0) | Track area in metres |
| `TRACK_WIDTH_M` | 0.020 m    | Line width |
| `QTR_CHANNELS` | 25         | Number of sensor channels |
| `MAX_WHEEL_SPEED` | 1.0 m/s    | Motor speed limit |
| `WHEEL_BASE` | 0.12 m     | Distance between wheels |
| `MU_SLIDE` | 1.14       | Sliding friction coefficient |
