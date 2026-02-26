#!/usr/bin/env python3
"""
Direct comparison: Run first 100 steps of both optimizer and visualization logic
and compare sensor readings to find divergence.
"""

import numpy as np
import sys
from config import *
from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from PIL import ImageFilter
import os

print("="*80)
print("DIRECT COMPARISON: First 100 steps of simulator")
print("="*80)

# Load track
track_filename = "bane_fase2.png"
track_path = os.path.join(os.path.dirname(__file__), '../..', 'assets', track_filename)
track = load_track_image(track_path)
blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
blur_arr = np.array(blurred, dtype=np.float32)

spawn = SPAWN_REGISTRY.get(track_filename)
x_start, y_start, theta_start = spawn["x"], spawn["y"], spawn["theta"]

print(f"Track: {track_filename}")
print(f"Spawn: ({x_start}, {y_start}), theta={theta_start}")
print(f"Parameters: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
print(f"DT={DT}, NOISE_SEED={NOISE_SEED}")
print()

# Simulation 1: Like optimizer (no matplotlib)
print("Simulation 1: Optimizer mode (no matplotlib)")
np.random.seed(NOISE_SEED)

robot1 = Robot()
robot1.position = np.array([x_start, y_start])
robot1.theta = theta_start
robot1.vL = 0.0
robot1.vR = 0.0

sensors1 = QTRArray()
pid1 = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD, limit=PID_LIMIT,
           integral_limit=PID_INTEGRAL_LIMIT, derivative_filter=PID_DERIV_FILTER)
pid1.reset()
sc1 = SpeedController(straight_speed=SC_STRAIGHT_SPEED, turn_speed=SC_TURN_SPEED,
                      error_threshold=SC_ERROR_THRESHOLD, smoothing=SC_SMOOTHING)
sc1.reset()

t1 = 0.0
last_e_y1 = 0.0
sensor_log1 = []

for step in range(100):
    readings = sensors1.read(robot1, blur_arr)
    sensor_log1.append(readings.copy())

    weights = readings ** 2
    pos_y = sensors1.sensor_pos_body[:, 1]
    total_w = weights.sum()

    if total_w > LINE_THRESH:
        e_y = float(np.dot(weights, pos_y) / total_w)
        last_e_y1 = e_y
    else:
        e_y = last_e_y1

    w_cmd = pid1.compute(e_y, DT)
    v_cmd = sc1.update(e_y, w_cmd, pid1.limit)
    vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
    vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

    robot1.update(vL_cmd, vR_cmd)
    t1 += DT

print(f"Completed 100 steps, t={t1:.3f}s")
print(f"Final position: ({robot1.position[0]:.6f}, {robot1.position[1]:.6f})")
print(f"Final theta: {robot1.theta:.6f}")
print()

# Simulation 2: Like visualization (with matplotlib import)
print("Simulation 2: Visualization mode (with matplotlib)")

# Import matplotlib AFTER setting seed (like our fix)
np.random.seed(NOISE_SEED)
import matplotlib
matplotlib.use("TkAgg")

robot2 = Robot()
robot2.position = np.array([x_start, y_start])
robot2.theta = theta_start
robot2.vL = 0.0
robot2.vR = 0.0

sensors2 = QTRArray()
pid2 = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD, limit=PID_LIMIT,
           integral_limit=PID_INTEGRAL_LIMIT, derivative_filter=PID_DERIV_FILTER)
pid2.reset()
sc2 = SpeedController(straight_speed=SC_STRAIGHT_SPEED, turn_speed=SC_TURN_SPEED,
                      error_threshold=SC_ERROR_THRESHOLD, smoothing=SC_SMOOTHING)
sc2.reset()

t2 = 0.0
last_e_y2 = 0.0
sensor_log2 = []

for step in range(100):
    readings = sensors2.read(robot2, blur_arr)
    sensor_log2.append(readings.copy())

    weights = readings ** 2
    pos_y = sensors2.sensor_pos_body[:, 1]
    total_w = weights.sum()

    if total_w > LINE_THRESH:
        e_y = float(np.dot(weights, pos_y) / total_w)
        last_e_y2 = e_y
    else:
        e_y = last_e_y2

    w_cmd = pid2.compute(e_y, DT)
    v_cmd = sc2.update(e_y, w_cmd, pid2.limit)
    vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
    vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

    robot2.update(vL_cmd, vR_cmd)
    t2 += DT

print(f"Completed 100 steps, t={t2:.3f}s")
print(f"Final position: ({robot2.position[0]:.6f}, {robot2.position[1]:.6f})")
print(f"Final theta: {robot2.theta:.6f}")
print()

# Compare
print("="*80)
print("COMPARISON:")
print("="*80)

pos_diff = np.linalg.norm(robot1.position - robot2.position)
theta_diff = abs(robot1.theta - robot2.theta)

print(f"Position difference: {pos_diff:.10f} m")
print(f"Theta difference: {theta_diff:.10f} rad")
print()

# Compare sensor readings
diverged = False
divergence_step = -1
for step in range(100):
    diff = np.linalg.norm(sensor_log1[step] - sensor_log2[step])
    if diff > 1e-10:
        print(f"✗ SENSOR DIVERGENCE at step {step}!")
        print(f"  Sensor diff: {diff:.10f}")
        print(f"  Reading 1: {sensor_log1[step][:3]}")
        print(f"  Reading 2: {sensor_log2[step][:3]}")
        diverged = True
        divergence_step = step
        break

if not diverged:
    print("✓ NO DIVERGENCE in first 100 steps - sensors are identical!")
    print("  The matplotlib import fix is working correctly.")
    print()
    print("  If full simulation still fails, the issue occurs later.")
    print("  Check if visualization rendering affects physics state.")
else:
    print()
    print("✗ DIVERGENCE DETECTED - matplotlib or import order still causing issues")

print("="*80)

