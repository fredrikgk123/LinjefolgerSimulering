#!/usr/bin/env python3
"""
Simple test: Run both simulation modes WITHOUT visualization and compare.
This eliminates matplotlib as a variable.
"""

import numpy as np
from config import *
from lap_optimizer import run_lap

# Parameters
track_filename = "bane_fase2.png"
params = {
    'kp': PID_KP,
    'ki': PID_KI,
    'kd': PID_KD,
    'pid_limit': PID_LIMIT,
    'straight_speed': SC_STRAIGHT_SPEED,
    'turn_speed': SC_TURN_SPEED,
    'error_threshold': SC_ERROR_THRESHOLD,
    'smoothing': SC_SMOOTHING,
}

print("Testing with parameters from config.py:")
print(f"  PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, Limit={PID_LIMIT}")
print(f"  Speed: Straight={SC_STRAIGHT_SPEED}, Turn={SC_TURN_SPEED}")
print(f"  Error Threshold: {SC_ERROR_THRESHOLD}")
print(f"  DT: {DT}, NOISE_SEED: {NOISE_SEED}")
print()

# Run multiple times with same seed to verify determinism
for i in range(3):
    np.random.seed(NOISE_SEED)
    lap_time, valid, max_error = run_lap(track_filename, params, show_viz=False)
    status = "COMPLETE" if (lap_time is not None and valid) else "DNF"
    print(f"Run {i+1}: {status}, lap_time={lap_time}, max_error={max_error*1000:.2f}mm")

print("\nIf all 3 runs show identical results, the optimizer is deterministic.")
print("If results vary, there's a randomness issue.")

