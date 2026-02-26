#!/bin/bash
# Simple shell script to test both modes quickly

echo "========================================================================"
echo "QUICK SYNCHRONIZATION TEST"
echo "========================================================================"
echo ""

echo "1. Testing optimizer (3 seconds timeout)..."
timeout 3 python3 -c "
from lap_optimizer import run_lap
from config import *
params = {
    'kp': PID_KP, 'ki': PID_KI, 'kd': PID_KD, 'pid_limit': PID_LIMIT,
    'straight_speed': SC_STRAIGHT_SPEED, 'turn_speed': SC_TURN_SPEED,
    'error_threshold': SC_ERROR_THRESHOLD, 'smoothing': SC_SMOOTHING
}
lap_time, valid, _ = run_lap('bane_fase2.png', params, show_viz=False)
print(f'Optimizer: lap_time={lap_time}, valid={valid}')
" || echo "Optimizer timed out or failed"

echo ""
echo "2. Now run visualization manually:"
echo "   python3 main.py --track ../assets/bane_fase2.png"
echo ""
echo "Compare the results!"
echo "========================================================================"

