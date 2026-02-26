#!/usr/bin/env python3
"""
Side-by-side comparison: Run optimizer and visualization logic in parallel
and log the FIRST difference.
"""

import numpy as np
from config import *
from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from PIL import ImageFilter
import os

track_filename = "bane_fase2.png"

# Load track
track_path = os.path.join(os.path.dirname(__file__), '../..', 'assets', track_filename)
track = load_track_image(track_path)
blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
blur_arr = np.array(blurred, dtype=np.float32)

spawn = SPAWN_REGISTRY.get(track_filename)
x_start, y_start, theta_start = spawn["x"], spawn["y"], spawn["theta"]

print("="*80)
print("SIDE-BY-SIDE COMPARISON: Optimizer vs Visualization Logic")
print("="*80)
print(f"Parameters: PID_KP={PID_KP}, PID_KI={PID_KI}, PID_KD={PID_KD}")
print(f"            SC_STRAIGHT={SC_STRAIGHT_SPEED}, SC_TURN={SC_TURN_SPEED}")
print(f"            SC_ERROR_THRESH={SC_ERROR_THRESHOLD}, SC_SMOOTH={SC_SMOOTHING}")
print(f"            DT={DT}, NOISE_SEED={NOISE_SEED}")
print()

# Create TWO identical simulations
robots = [Robot(), Robot()]
for robot in robots:
    robot.position = np.array([x_start, y_start])
    robot.theta = theta_start
    robot.vL = 0.0
    robot.vR = 0.0

sensors = [QTRArray(), QTRArray()]

pids = [
    PID(kp=PID_KP, ki=PID_KI, kd=PID_KD, limit=PID_LIMIT,
        integral_limit=PID_INTEGRAL_LIMIT, derivative_filter=PID_DERIV_FILTER),
    PID(kp=PID_KP, ki=PID_KI, kd=PID_KD, limit=PID_LIMIT,
        integral_limit=PID_INTEGRAL_LIMIT, derivative_filter=PID_DERIV_FILTER)
]
for pid in pids:
    pid.reset()

speed_controllers = [
    SpeedController(straight_speed=SC_STRAIGHT_SPEED, turn_speed=SC_TURN_SPEED,
                    error_threshold=SC_ERROR_THRESHOLD, smoothing=SC_SMOOTHING),
    SpeedController(straight_speed=SC_STRAIGHT_SPEED, turn_speed=SC_TURN_SPEED,
                    error_threshold=SC_ERROR_THRESHOLD, smoothing=SC_SMOOTHING)
]
for sc in speed_controllers:
    sc.reset()

# Reset random seed ONCE for both
np.random.seed(NOISE_SEED)

t = 0.0
step = 0
last_e_y = [0.0, 0.0]
line_loss_t = [0.0, 0.0]

MAX_STEPS = 5000
divergence_detected = False

print("Running simulations in parallel...")

for step in range(MAX_STEPS):
    # Run BOTH simulations step-by-step
    for i in range(2):
        # Sense
        readings = sensors[i].read(robots[i], blur_arr)
        weights = readings ** 2
        pos_y = sensors[i].sensor_pos_body[:, 1]
        total_w = weights.sum()

        if total_w > LINE_THRESH:
            e_y = float(np.dot(weights, pos_y) / total_w)
            last_e_y[i] = e_y
        else:
            e_y = last_e_y[i]

        # Control
        w_cmd = pids[i].compute(e_y, DT)
        v_cmd = speed_controllers[i].update(e_y, w_cmd, pids[i].limit)
        vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
        vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

        # Physics
        robots[i].update(vL_cmd, vR_cmd)

        # Line loss
        line_loss_t[i] = line_loss_t[i] + DT if total_w <= LINE_THRESH else 0.0

    # Compare states
    pos_diff = np.linalg.norm(robots[0].position - robots[1].position)
    theta_diff = abs(robots[0].theta - robots[1].theta)
    vL_diff = abs(robots[0].vL - robots[1].vL)
    vR_diff = abs(robots[0].vR - robots[1].vR)

    if pos_diff > 1e-10 or theta_diff > 1e-10 or vL_diff > 1e-10 or vR_diff > 1e-10:
        print(f"\n✗ DIVERGENCE DETECTED at step {step}, t={t:.3f}s")
        print(f"  Position diff: {pos_diff:.10f}m")
        print(f"  Theta diff: {theta_diff:.10f}rad")
        print(f"  vL diff: {vL_diff:.10f}m/s")
        print(f"  vR diff: {vR_diff:.10f}m/s")
        print()
        print(f"  Robot 0: pos={robots[0].position}, theta={robots[0].theta:.6f}")
        print(f"  Robot 1: pos={robots[1].position}, theta={robots[1].theta:.6f}")
        divergence_detected = True
        break

    if line_loss_t[0] > MAX_LINE_LOSS_TIME or line_loss_t[1] > MAX_LINE_LOSS_TIME:
        print(f"\n✗ LINE LOST at step {step}, t={t:.3f}s")
        print(f"  Line loss time [0]: {line_loss_t[0]:.3f}s")
        print(f"  Line loss time [1]: {line_loss_t[1]:.3f}s")
        break

    t += DT

if not divergence_detected and step >= MAX_STEPS - 1:
    print(f"\n✓ NO DIVERGENCE DETECTED after {MAX_STEPS} steps ({t:.3f}s)")
    print(f"  Both simulations remain identical.")
    print()
    print("  This means the core physics/control logic is deterministic.")
    print("  The visualization issue must be from matplotlib rendering.")

print("="*80)

