# main.py

import numpy as np
from PIL import ImageFilter
import matplotlib
matplotlib.use("TkAgg")

from track.track_generator import generate_default_track
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID
from visualization.plots import setup_realtime_plot, plot_results
from config import *


def main():
    # ── Track ────────────────────────────────────────────────────────────────
    track   = generate_default_track()
    blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    # ── Robot start position ──────────────────────────────────────────────────
    # Place robot at the FIRST PEAK of the sine track (slope = 0, heading = 0°).
    # Track: y = 0.5H + 0.25H·sin(ω·x),  ω = 6π/W
    # Peak at ω·x = π/2  →  x = W/12
    W, H = MAP_SIZE_M
    omega = 3.0 / W * 2 * np.pi          # same formula as track_generator
    x_start = (np.pi / 2) / omega        # = W/12 ≈ 0.25 m
    y_start = 0.5 * H + 0.25 * H        # = 1.5 m  (peak, sin=1)
    theta_start = 0.0                    # horizontal — slope is 0 at peak

    robot = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta    = theta_start

    sensors = QTRArray()

    # ── PID ───────────────────────────────────────────────────────────────────
    # e_y > 0 → line is to robot's LEFT  → steer LEFT  → positive ω
    # Differential drive:  w = (vR-vL)/WB,  vL = v - w·WB/2,  vR = v + w·WB/2
    # So pid output directly becomes angular velocity command.
    pid = PID(kp=5.0, ki=0.5, kd=0.15,
              limit=8.0, integral_limit=1.0, derivative_filter=0.2)

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

        # Reduce speed on sharp turns to avoid overshoot
        # Adaptive speed: slow down more aggressively on sharp turns
        turn_factor = abs(w_cmd) / pid.limit
        v_cmd = 0.40 * max(0.30, 1.0 - 0.7 * turn_factor)

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
