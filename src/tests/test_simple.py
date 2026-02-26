#!/usr/bin/env python3
"""
Simple consistency test - minimal imports
"""
import sys
import os

# Add parent directory (src/) to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

# Set matplotlib backend before any imports
import matplotlib
matplotlib.use('Agg')

from lap_optimizer import run_lap
from config import PID_KP, PID_KI, PID_KD, PID_LIMIT
from config import SC_STRAIGHT_SPEED, SC_TURN_SPEED, SC_ERROR_THRESHOLD, SC_SMOOTHING

# Test with the current config parameters
params = {
    'kp': PID_KP,
    'ki': PID_KI,
    'kd': PID_KD,
    'pid_limit': PID_LIMIT,
    'straight_speed': SC_STRAIGHT_SPEED,
    'turn_speed': SC_TURN_SPEED,
    'error_threshold': SC_ERROR_THRESHOLD,
    'smoothing': SC_SMOOTHING
}

print('='*70)
print('CONSISTENCY TEST: lap_optimizer.run_lap()')
print('='*70)
print(f'Track: bane_fase2.png')
print(f'Kp={params["kp"]:.1f}, Ki={params["ki"]:.2f}, Kd={params["kd"]:.1f}, limit={params["pid_limit"]:.1f}')
print('='*70)

# Run multiple times to test consistency
results = []
print('Running 3 simulations...')
for i in range(3):
    print(f'  Run {i+1}...', end='', flush=True)
    lap_time, valid, max_error = run_lap('bane_fase2.png', params, show_viz=False)
    results.append((lap_time, valid, max_error))
    status = 'VALID' if valid else 'INVALID'
    time_str = f'{lap_time:.3f}s' if lap_time is not None else 'DNF'
    print(f' {status:8s}  lap_time={time_str:8s}  max_error={max_error:.4f}m')

# Check consistency
print('='*70)
lap_times = [r[0] for r in results]
valid_flags = [r[1] for r in results]

if all(lap_times[0] == t for t in lap_times) and all(valid_flags[0] == v for v in valid_flags):
    print('✅ CONSISTENT: All runs produced identical results')
else:
    print('❌ INCONSISTENT: Runs produced different results')
    print(f'   Lap times: {lap_times}')
    print(f'   Valid flags: {valid_flags}')
print('='*70)

