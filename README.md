# Line-Following Robot Simulation

![Simulation Output](assets/for_readme/image1.png)
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
│   ├── optimize.py                ← PID optimizer CLI
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
│   │   └── multi_track.py         ← 5 built-in tracks for optimizer testing
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
│       └── image1.png
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

**Hardcoding a spawn point** for a known track (so the robot starts exactly on the line):  
Open `src/main.py` and add an entry to `SPAWN_REGISTRY`:
```python
SPAWN_REGISTRY = {
    "suzuka.png":     {"x": 0.55, "y": 1.60, "theta": 0.0},
    "bane_fase2.png": {"x": 0.50, "y": 1.00, "theta": 0.0},
    "my_track.png":   {"x": 0.20, "y": 0.80, "theta": 1.57},  # ← add yours
}
```
`x`/`y` are in metres (world coordinates, origin = bottom-left), `theta` in radians (`0` = pointing right, `π/2` = pointing up).

---

## Run PID Optimizer

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

Runs your current PID on 5 different track geometries and saves comparison plots to `output/`:

| Plot | Description |
|------|-------------|
| `multi_track_results.png` | Stacked list with EXCELLENT / GOOD / OK / POOR rating per track |
| `multi_track_bars.png` | Side-by-side bar charts (Max error, RMS, Settling time) |
| `multi_track_table.png` | Summary table with aggregated averages |

---

## Configuration

All key parameters live in `src/config.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SIM_TIME` | 15.0 s | Simulation duration |
| `DT` | 0.005 s | Physics timestep |
| `MAP_SIZE_M` | (4.0, 2.0) | Track area in metres |
| `TRACK_WIDTH_M` | 0.020 m | Line width |
| `QTR_CHANNELS` | 25 | Number of sensor channels |
| `MAX_WHEEL_SPEED` | 1.0 m/s | Motor speed limit |
| `WHEEL_BASE` | 0.12 m | Distance between wheels |
| `MU_SLIDE` | 1.14 | Sliding friction coefficient |
