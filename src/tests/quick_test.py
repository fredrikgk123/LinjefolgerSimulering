#!/usr/bin/env python3
"""
Quick test with your exact parameters to see if there's still desynchronization.
"""

import numpy as np
from config import *
from lap_optimizer import run_lap

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

print("Quick test with parameters from config.py:")
print(f"  PID_KP={PID_KP}, PID_KI={PID_KI}, PID_KD={PID_KD}")
print(f"  SC_STRAIGHT_SPEED={SC_STRAIGHT_SPEED}, SC_TURN_SPEED={SC_TURN_SPEED}")
print(f"  SC_ERROR_THRESHOLD={SC_ERROR_THRESHOLD}")
print()

print("Running optimizer (no visualization)...")
lap_time, valid, max_error = run_lap('bane_fase2.png', params, show_viz=False)

if lap_time is not None:
    print(f"✓ COMPLETE: lap_time={lap_time:.3f}s, valid={valid}, max_error={max_error*1000:.2f}mm")
else:
    print(f"✗ DNF: valid={valid}, max_error={max_error*1000:.2f}mm")

print()
print("Now run: python3 main.py --track ../assets/bane_fase2.png")
print(f"Expected result: lap_time ≈ {lap_time:.3f}s (±0.01s)")

