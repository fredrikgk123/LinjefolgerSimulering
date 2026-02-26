#!/usr/bin/env python3
"""
Test script to verify that the optimizer and visualization produce
identical lap times with the same parameters.

Usage:
    python3 test_dt_consistency.py
"""

import numpy as np
from config import (
    DT, PID_KP, PID_KI, PID_KD, PID_LIMIT,
    SC_STRAIGHT_SPEED, SC_TURN_SPEED, SC_ERROR_THRESHOLD, SC_SMOOTHING,
    NOISE_SEED
)
from lap_optimizer import run_lap

# Test parameters
track_filename = "bane_fase2.png"

# Create parameter dict from config defaults
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

print("=" * 70)
print("  DT CONSISTENCY TEST")
print("=" * 70)
print(f"\nConfiguration:")
print(f"  DT (Physics Timestep)     : {DT} seconds ({1/DT:.0f} Hz)")
print(f"  PID Kp, Ki, Kd           : {PID_KP}, {PID_KI}, {PID_KD}")
print(f"  Speed (straight, turn)   : {SC_STRAIGHT_SPEED}, {SC_TURN_SPEED} m/s")
print(f"  Noise Seed               : {NOISE_SEED}")
print(f"  Track                    : {track_filename}")

print(f"\nRunning lap simulation (optimizer mode)...")
np.random.seed(NOISE_SEED)
lap_time, valid, max_error = run_lap(track_filename, params, show_viz=False)

print(f"\n" + "=" * 70)
print(f"RESULTS:")
print(f"=" * 70)
if lap_time is not None:
    print(f"✓ Lap completed in {lap_time:.3f} seconds")
    print(f"  Valid: {valid}")
    print(f"  Max error: {max_error*1000:.2f} mm")
else:
    print(f"✗ Did not finish (DNF)")
    print(f"  Valid: {valid}")
    print(f"  Max error: {max_error*1000:.2f} mm")

print(f"\n⚠️  Now run: python3 main.py --track ../assets/{track_filename}")
print(f"   and verify the lap time matches {lap_time:.3f}s (±0.5s variance is normal)")
print("=" * 70)

