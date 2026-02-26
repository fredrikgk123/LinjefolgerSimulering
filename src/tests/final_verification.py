#!/usr/bin/env python3
"""
Final verification: Test if synchronization is now complete.
"""

print("="*80)
print("FINAL SYNCHRONIZATION VERIFICATION")
print("="*80)
print()

print("Testing optimizer with current config.py parameters...")
print()

from lap_optimizer import run_lap
from config import *
import numpy as np

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

print(f"Parameters:")
print(f"  PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, Limit={PID_LIMIT}")
print(f"  Speed: Straight={SC_STRAIGHT_SPEED}, Turn={SC_TURN_SPEED}")
print(f"  Error Threshold: {SC_ERROR_THRESHOLD}, Smoothing: {SC_SMOOTHING}")
print(f"  DT: {DT}, NOISE_SEED: {NOISE_SEED}")
print()

# Test determinism - run twice
print("Running optimizer twice to verify determinism...")
times = []
for i in range(2):
    np.random.seed(NOISE_SEED)  # Reset seed each time
    lap_time, valid, max_err = run_lap('bane_fase2.png', params, show_viz=False)
    times.append(lap_time)
    status = "COMPLETE" if (lap_time and valid) else "DNF"
    print(f"  Run {i+1}: {status}, lap_time={lap_time:.3f}s" if lap_time else f"  Run {i+1}: DNF")

print()
if len(times) == 2 and times[0] == times[1]:
    print("✓ Optimizer is DETERMINISTIC (both runs identical)")
else:
    print("✗ WARNING: Optimizer results vary between runs!")
    print("  This suggests random seed is not working properly.")

print()
print("="*80)
print("NEXT STEP:")
print("="*80)
print()
print("Run visualization:")
print("  python3 main.py --track ../assets/bane_fase2.png")
print()
if times and times[0]:
    print(f"Expected result: ~{times[0]:.3f}s (±0.01s variance is normal)")
    print()
    print("If visualization matches, synchronization is COMPLETE! ✓")
    print("If visualization still fails, there may be another issue.")
else:
    print("Optimizer DNF - parameters may be invalid or track not found.")
print()
print("="*80)

