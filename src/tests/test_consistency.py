#!/usr/bin/env python3
"""
Test script to verify consistency between optimizer and main simulation.
"""

import sys
import os
# Add src/ to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lap_optimizer import run_lap
from config import *

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
print(f'Parameters: Kp={params["kp"]:.1f}, Ki={params["ki"]:.2f}, Kd={params["kd"]:.1f}')
print(f'            limit={params["pid_limit"]:.1f}')
print(f'            straight_speed={params["straight_speed"]:.3f}, turn_speed={params["turn_speed"]:.3f}')
print(f'            error_threshold={params["error_threshold"]:.4f}, smoothing={params["smoothing"]:.3f}')
print('='*70)

# Run multiple times to test consistency
results = []
for i in range(3):
    lap_time, valid, max_error = run_lap('bane_fase2.png', params, show_viz=False)
    results.append((lap_time, valid, max_error))
    status = 'VALID' if valid else 'INVALID'
    time_str = f'{lap_time:.3f}s' if lap_time is not None else 'DNF'
    print(f'  Run {i+1}: {status:8s}  lap_time={time_str:8s}  max_error={max_error:.4f}m')

# Check consistency
print('='*70)
if len(results) >= 2:
    lap_times = [r[0] for r in results]
    valid_flags = [r[1] for r in results]

    if all(lap_times[0] == t for t in lap_times) and all(valid_flags[0] == v for v in valid_flags):
        print('✅ CONSISTENT: All runs produced identical results')
    else:
        print('❌ INCONSISTENT: Runs produced different results')
        print(f'   Lap times: {lap_times}')
        print(f'   Valid flags: {valid_flags}')
else:
    print('⚠️  Not enough runs to check consistency')
print('='*70)

