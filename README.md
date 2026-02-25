# Line-Following Robot Simulation

![Simulation Output](assets/for_readme/image1.png)
*Live simulation view (left) showing the robot tracking a sine wave track, with real-time sensor array readings, lateral error, and wheel speed plots (right).*

## Structure

```
Simulering/
├── src/                  ← All source code
│   ├── main.py           ← Entry point
│   ├── config.py         ← Simulation parameters
│   ├── optimize.py       ← PID optimizer entry point
│   ├── pid_optimizer.py  ← ML optimization engine
│   ├── multi_track_simulator.py
│   ├── multi_track_plots.py
│   ├── performance_metrics.py
│   ├── control/          ← PID controller + state machine
│   ├── physics/          ← Robot model + friction
│   ├── sensors/          ← QTR sensor array
│   ├── track/            ← Track generators
│   ├── visualization/    ← Plots + overlays
│   └── tests/            ← Test and debug scripts
├── docs/                 ← Documentation
├── output/               ← Generated images and results
└── assets/               ← Static files (track images)
```

## Run Simulation

```bash
cd src

python3 main.py                                        # default sine-wave track
python3 main.py --track ../assets/suzuka.png           # Suzuka circuit
python3 main.py --track ../assets/my_track.png         # any track image you add
```

> **Adding your own track:** drop any top-down track image (PNG/JPG) into `assets/` and pass it with `--track`. The robot spawns in the centre and searches for the line.  
> Image requirements: dark line on a light background, top-down view.

## Run PID Optimizer

```bash
cd src

python3 optimize.py --mode quick       # ~13 min  | 27 combinations  | recommended first run
python3 optimize.py --mode bayesian    # ~75 min  | 30 random samples | good for refinement
python3 optimize.py --mode bayesian --iterations 100  # ~4 hrs | more samples
python3 optimize.py --mode full        # 50+ hrs  | 6400 combinations | exhaustive
```

## Run Multi-Track Test

```bash
cd src/tests
python3 test_multi_track.py
```

