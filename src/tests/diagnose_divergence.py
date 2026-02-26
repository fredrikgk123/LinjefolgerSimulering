#!/usr/bin/env python3
"""
Diagnostic script to compare optimizer vs visualization behavior step-by-step.
This will help identify where the divergence occurs.
"""

import numpy as np
from config import *
from lap_optimizer import run_lap
from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from PIL import ImageFilter

# Test parameters
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

print("=" * 70)
print("  DIAGNOSTIC: Optimizer vs Visualization Comparison")
print("=" * 70)
print(f"\nParameters:")
print(f"  PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, Limit={PID_LIMIT}")
print(f"  Speed: Straight={SC_STRAIGHT_SPEED}, Turn={SC_TURN_SPEED}")
print(f"  Error Threshold: {SC_ERROR_THRESHOLD}")
print(f"  Smoothing: {SC_SMOOTHING}")
print(f"  DT: {DT}")
print(f"  NOISE_SEED: {NOISE_SEED}")

# Run optimizer mode
print(f"\n{'='*70}")
print("OPTIMIZER MODE:")
print(f"{'='*70}")
np.random.seed(NOISE_SEED)
lap_time_opt, valid_opt, max_error_opt = run_lap(track_filename, params, show_viz=False)
print(f"Result: lap_time={lap_time_opt}, valid={valid_opt}, max_error={max_error_opt*1000:.2f}mm")

# Run visualization mode (without display)
print(f"\n{'='*70}")
print("VISUALIZATION MODE (no display):")
print(f"{'='*70}")

# Load track
import os
track_path = os.path.join(os.path.dirname(__file__), '../..', 'assets', track_filename)
track = load_track_image(track_path)
blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
blur_arr = np.array(blurred, dtype=np.float32)

# Spawn
spawn = SPAWN_REGISTRY.get(track_filename, {"x": 2.0, "y": 1.0, "theta": 0.0})
x_start, y_start, theta_start = spawn["x"], spawn["y"], spawn["theta"]

# Initialize robot
robot = Robot()
robot.position = np.array([x_start, y_start])
robot.theta = theta_start
robot.vL = 0.0
robot.vR = 0.0
sensors = QTRArray()

# Initialize controllers
pid = PID(
    kp=PID_KP, ki=PID_KI, kd=PID_KD,
    limit=PID_LIMIT,
    integral_limit=PID_INTEGRAL_LIMIT,
    derivative_filter=PID_DERIV_FILTER,
)
pid.reset()

speed_controller = SpeedController(
    straight_speed=SC_STRAIGHT_SPEED,
    turn_speed=SC_TURN_SPEED,
    error_threshold=SC_ERROR_THRESHOLD,
    smoothing=SC_SMOOTHING,
)
speed_controller.reset()

# Checkpoints
checkpoints = CHECKPOINT_REGISTRY.get(track_filename, [])
cp_next = 0
cp_total = len(checkpoints)

# Lap timer
lap_started = False
lap_time = None
lap_start_t = 0.0
last_e_y = 0.0
line_loss_t = 0.0

# Set random seed
np.random.seed(NOISE_SEED)
t = 0.0
step = 0

print(f"Starting simulation...")
print(f"Spawn: ({x_start}, {y_start}), theta={theta_start}")

divergence_detected = False
divergence_step = -1

while t < SIM_TIME:
    # Sense
    readings = sensors.read(robot, blur_arr)
    weights = readings ** 2
    pos_y = sensors.sensor_pos_body[:, 1]
    total_w = weights.sum()

    if total_w > LINE_THRESH:
        e_y = float(np.dot(weights, pos_y) / total_w)
        last_e_y = e_y
    else:
        e_y = last_e_y

    # Control
    w_cmd = pid.compute(e_y, DT)
    v_cmd = speed_controller.update(e_y, w_cmd, pid.limit)
    vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
    vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

    # Physics
    robot.update(vL_cmd, vR_cmd)

    rx, ry = robot.position
    dist_to_start = np.hypot(rx - x_start, ry - y_start)

    # Checkpoints
    if cp_next < cp_total:
        cx, cy = checkpoints[cp_next]
        if np.hypot(rx - cx, ry - cy) < CHECKPOINT_RADIUS:
            print(f"  Checkpoint {cp_next + 1}/{cp_total} cleared at t={t:.2f}s")
            cp_next += 1

    # Lap timer
    if not lap_started and dist_to_start > MIN_DEPARTURE_DIST:
        lap_started = True
        lap_start_t = t
        print(f"  Lap started at t={t:.2f}s")
    elif lap_started and lap_time is None and dist_to_start < START_FINISH_RADIUS:
        if cp_next < cp_total:
            print(f"  FALSE LAP — only {cp_next}/{cp_total} checkpoints cleared")
            break
        else:
            lap_time = t - lap_start_t
            print(f"  LAP COMPLETE at t={t:.2f}s, lap_time={lap_time:.3f}s")
            break

    # Line-loss guard
    line_loss_t = line_loss_t + DT if total_w <= LINE_THRESH else 0.0
    if line_loss_t > MAX_LINE_LOSS_TIME:
        print(f"\n*** LINE LOST at t={t:.2f}s (after {line_loss_t:.3f}s of continuous loss) ***")
        print(f"  Position: ({rx:.3f}, {ry:.3f})")
        print(f"  Heading: {robot.theta:.3f} rad")
        print(f"  Last error: {e_y*1000:.2f}mm")
        print(f"  Total sensor weight: {total_w:.4f}")
        divergence_detected = True
        divergence_step = step
        break

    t += DT
    step += 1

if lap_time is not None:
    print(f"\nResult: lap_time={lap_time:.3f}s, valid=True")
else:
    print(f"\nResult: DNF (line lost or incomplete)")

# Compare
print(f"\n{'='*70}")
print("COMPARISON:")
print(f"{'='*70}")
print(f"Optimizer:     lap_time={lap_time_opt}, valid={valid_opt}")
print(f"Visualization: lap_time={lap_time}, valid={lap_time is not None}")

if lap_time_opt is not None and lap_time is not None:
    diff = abs(lap_time_opt - lap_time)
    print(f"\nDifference: {diff:.3f}s ({diff/lap_time_opt*100:.2f}%)")
    if diff < 0.01:
        print("✓ SYNCED - Excellent match!")
    elif diff < 0.1:
        print("✓ ACCEPTABLE - Small variance")
    else:
        print("✗ DIVERGED - Significant difference!")
elif lap_time_opt is not None and lap_time is None:
    print("\n✗ CRITICAL DIVERGENCE:")
    print(f"  Optimizer completed in {lap_time_opt:.3f}s")
    print(f"  Visualization lost line at t={t:.2f}s")
    print(f"\nThis indicates the two modes are NOT synchronized!")
elif lap_time_opt is None and lap_time is not None:
    print("\n✗ CRITICAL DIVERGENCE:")
    print(f"  Optimizer DNF")
    print(f"  Visualization completed in {lap_time:.3f}s")
else:
    print("\nBoth DNF - check if parameters are valid")

print(f"{'='*70}")

