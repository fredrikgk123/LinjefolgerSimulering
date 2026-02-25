# main.py

import numpy as np
from PIL import ImageFilter
import matplotlib
matplotlib.use("TkAgg")

from track.track_generator import generate_default_track
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from visualization.plots import setup_realtime_plot, plot_results
from performance_metrics import analyze_tracking_performance, print_performance_report
from config import *


def main():
    # ── Track ────────────────────────────────────────────────────────────────
    track   = generate_default_track()
    blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    # ── Robot start position ──────────────────────────────────────────────────
    # Place robot at track start: x = 0.05*W
    # Track: y = 0.5H + 0.25H·sin(3·x/W·2π)
    # Calculate actual track position at start
    W, H = MAP_SIZE_M
    x_start = 0.05 * W                          # track starts at 0.05W = 0.15m
    y_start = 0.5 * H + 0.25 * H * np.sin(3 * x_start / W * 2 * np.pi)  # Actual track Y position

    # Calculate heading angle to be tangent to track
    # dy/dx = 0.25H · 3·2π/W · cos(3·x/W·2π)
    omega = 3.0 * 2 * np.pi / W
    slope = 0.25 * H * omega * np.cos(omega * x_start)
    theta_start = np.arctan(slope)

    robot = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta    = theta_start

    sensors = QTRArray()

    # ── PID ───────────────────────────────────────────────────────────────────
    # e_y > 0 → line is to robot's LEFT  → steer LEFT  → positive ω
    # Differential drive:  w = (vR-vL)/WB,  vL = v - w·WB/2,  vR = v + w·WB/2
    # So pid output directly becomes angular velocity command.
    # Aggressive tuning: fast turns, strong recovery, minimal line loss
    pid = PID(kp=100.0, ki=3.5, kd=16.0,
              limit=18.0, integral_limit=1.2, derivative_filter=0.10)

    # ── Speed Controller (State Machine) ──────────────────────────────────────
    # Increases speed when robot is on straight sections (low error, low turning)
    # Reduces speed during turns for stability
    speed_controller = SpeedController(
        straight_speed=0.90,      # High speed on straight sections
        turn_speed=0.55,          # Moderate speed in turns for stability
        error_threshold=0.007,    # 7mm threshold for state switching
        smoothing=0.12            # Smooth transitions for stability
    )

    # ── Live visualisation ────────────────────────────────────────────────────
    update_plot, robot_artist = setup_realtime_plot(blur_arr)

    traj, err_log, sensor_log = [], [], []
    last_e_y   = 0.0
    LINE_THRESH = 0.08    # min Σ(weights) to trust centroid reading
    RENDER_EVERY = 8      # render every 8 steps → ~25 fps

    t    = 0.0
    step = 0

    while t < SIM_TIME:
        # ── Sense ─────────────────────────────────────────────────────────────
        readings = sensors.read(robot, blur_arr)

        # Weighted centroid in body-frame lateral axis
        weights = readings ** 2
        pos_y   = sensors.sensor_pos_body[:, 1]
        total_w = weights.sum()

        if total_w > LINE_THRESH:
            e_y      = float(np.dot(weights, pos_y) / total_w)
            last_e_y = e_y
        else:
            e_y = last_e_y          # hold last known error when line is lost

        # ── Control ───────────────────────────────────────────────────────────
        # e_y > 0 → line left → steer left (+w) → correct
        w_cmd = pid.compute(e_y, DT)

        # Use state machine for adaptive speed control
        v_cmd = speed_controller.update(e_y, w_cmd, pid.limit)

        vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
        vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

        # ── Physics step ──────────────────────────────────────────────────────
        robot.update(vL_cmd, vR_cmd)
        traj.append(tuple(robot.position))
        err_log.append(e_y)
        sensor_log.append(readings.copy())

        # ── Render ────────────────────────────────────────────────────────────
        if step % RENDER_EVERY == 0:
            update_plot(robot, readings, e_y, robot.vL, robot.vR, t)

        t    += DT
        step += 1

    # ── Summary ───────────────────────────────────────────────────────────────
    # Calculate and print performance metrics
    metrics = analyze_tracking_performance(err_log, DT)
    print_performance_report(metrics)

    plot_results(traj, sensor_log, err_log,
                 map_arr=np.array(blurred, dtype=np.uint8))


if __name__ == "__main__":
    main()
