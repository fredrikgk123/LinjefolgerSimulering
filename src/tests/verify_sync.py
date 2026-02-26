#!/usr/bin/env python3
"""
Quick verification: Test if optimizer and visualization are now synchronized.
"""

print("="*80)
print("SYNCHRONIZATION VERIFICATION TEST")
print("="*80)
print()

# Test optimizer
print("1. Testing optimizer (no visualization)...")
from lap_optimizer import run_lap
from config import *

params = {
    'kp': PID_KP, 'ki': PID_KI, 'kd': PID_KD, 'pid_limit': PID_LIMIT,
    'straight_speed': SC_STRAIGHT_SPEED, 'turn_speed': SC_TURN_SPEED,
    'error_threshold': SC_ERROR_THRESHOLD, 'smoothing': SC_SMOOTHING
}

lap_time, valid, max_error = run_lap('bane_fase2.png', params, show_viz=False)

if lap_time is not None and valid:
    print(f"   ✓ Optimizer: {lap_time:.3f}s (COMPLETE)")
else:
    print(f"   ✗ Optimizer: DNF (line lost or invalid)")

print()
print("2. Now run visualization:")
print("   python3 main.py --track ../assets/bane_fase2.png")
print()
print(f"3. Expected visualization result: {lap_time:.3f}s ±0.01s")
print()
print("If visualization matches optimizer within ±0.01s, synchronization is fixed!")
print("="*80)

