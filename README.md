# Line-Following Robot Simulation

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
python3 main.py
```

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

