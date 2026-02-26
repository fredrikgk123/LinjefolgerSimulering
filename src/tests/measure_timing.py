#!/usr/bin/env python3
"""
Test to measure if visualization actually runs at the expected timestep.
"""

import time
import numpy as np
from config import *
from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from visualization.plots import setup_realtime_plot
from PIL import ImageFilter
import os

track_filename = "bane_fase2.png"
track_path = os.path.join(os.path.dirname(__file__), '../..', 'assets', track_filename)
track = load_track_image(track_path)
blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
blur_arr = np.array(blurred, dtype=np.float32)

spawn = SPAWN_REGISTRY.get(track_filename)
x_start, y_start, theta_start = spawn["x"], spawn["y"], spawn["theta"]

robot = Robot()
robot.position = np.array([x_start, y_start])
robot.theta = theta_start
robot.vL = 0.0
robot.vR = 0.0
sensors = QTRArray()

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

update_plot, _ = setup_realtime_plot(
    blur_arr,
    spawn=(x_start, y_start),
    sf_radius=START_FINISH_RADIUS,
    checkpoints=CHECKPOINT_REGISTRY.get(track_filename, []),
)

np.random.seed(NOISE_SEED)
t = 0.0
step = 0
last_e_y = 0.0
RENDER_EVERY = 8

wall_times = []
sim_times = []

print("Running 200 steps to measure timing...")
print(f"Expected DT: {DT}s")
print(f"Render every: {RENDER_EVERY} steps")
print()

wall_start = time.time()

for step in range(200):
    step_start = time.time()

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

    # Render
    if step % RENDER_EVERY == 0:
        update_plot(robot, readings, e_y, robot.vL, robot.vR, t,
                    lap_time=None, elapsed=t, cp_cleared=0)

    t += DT

    step_end = time.time()
    step_duration = step_end - step_start
    wall_times.append(step_duration)
    sim_times.append(DT)

wall_end = time.time()
wall_total = wall_end - wall_start

print(f"{'='*70}")
print(f"TIMING ANALYSIS:")
print(f"{'='*70}")
print(f"Total simulation time: {t:.3f}s")
print(f"Total wall-clock time: {wall_total:.3f}s")
print(f"Real-time ratio: {wall_total/t:.2f}x (>1 means slower than real-time)")
print()
print(f"Average step duration: {np.mean(wall_times)*1000:.2f}ms")
print(f"Expected step duration: {DT*1000:.2f}ms")
print(f"Min step duration: {np.min(wall_times)*1000:.2f}ms")
print(f"Max step duration: {np.max(wall_times)*1000:.2f}ms")
print()

# Check if rendering steps take longer
render_steps = [wall_times[i] for i in range(len(wall_times)) if i % RENDER_EVERY == 0]
non_render_steps = [wall_times[i] for i in range(len(wall_times)) if i % RENDER_EVERY != 0]

if render_steps:
    print(f"Steps WITH rendering:    avg={np.mean(render_steps)*1000:.2f}ms, max={np.max(render_steps)*1000:.2f}ms")
if non_render_steps:
    print(f"Steps WITHOUT rendering: avg={np.mean(non_render_steps)*1000:.2f}ms, max={np.max(non_render_steps)*1000:.2f}ms")

print()
print("NOTE: If rendering steps take significantly longer, matplotlib is")
print("      causing timing delays that could desynchronize the simulation.")
print(f"{'='*70}")

