#!/usr/bin/env python3
"""
Test main.py logic WITHOUT visualization to isolate matplotlib as the issue.
"""

import numpy as np
from config import *
from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from PIL import ImageFilter
import os

print("="*80)
print("TEST: main.py logic WITHOUT visualization")
print("="*80)

# Load track
track_filename = "bane_fase2.png"
track_path = os.path.join(os.path.dirname(__file__), '../..', 'assets', track_filename)
track = load_track_image(track_path)
blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
blur_arr = np.array(blurred, dtype=np.float32)

spawn = SPAWN_REGISTRY.get(track_filename)
x_start, y_start, theta_start = spawn["x"], spawn["y"], spawn["theta"]

# Initialize robot
robot = Robot()
robot.position = np.array([x_start, y_start])
robot.theta = theta_start
robot.vL = 0.0
robot.vR = 0.0
sensors = QTRArray()

# Initialize controllers
pid = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD, limit=PID_LIMIT,
          integral_limit=PID_INTEGRAL_LIMIT, derivative_filter=PID_DERIV_FILTER)
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

print(f"Parameters: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, Limit={PID_LIMIT}")
print(f"Speed: Straight={SC_STRAIGHT_SPEED}, Turn={SC_TURN_SPEED}")
print(f"DT={DT}, NOISE_SEED={NOISE_SEED}")
print(f"Starting simulation (NO visualization)...")
print()

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
            print(f"  LAP COMPLETE at t={t:.2f}s")
            break

    # Line-loss guard
    line_loss_t = line_loss_t + DT if total_w <= LINE_THRESH else 0.0
    if line_loss_t > MAX_LINE_LOSS_TIME:
        print(f"\n*** LINE LOST at t={t:.2f}s ***")
        print(f"  Position: ({rx:.3f}, {ry:.3f})")
        print(f"  Last error: {e_y*1000:.2f}mm")
        break

    t += DT
    step += 1

print()
print("="*80)
print("RESULT:")
print("="*80)
if lap_time is not None:
    print(f"✓ LAP COMPLETE: {lap_time:.3f}s")
else:
    print(f"✗ DNF: Line lost at t={t:.2f}s")

print()
print("This simulates main.py WITHOUT matplotlib rendering.")
print("If this completes but full main.py fails, visualization is the issue.")
print("="*80)

