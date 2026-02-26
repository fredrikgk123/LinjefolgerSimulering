# main.py

# CRITICAL FIX: Set random seed at the VERY FIRST import!
# plots.py imports matplotlib, so we must set seed before importing plots
import numpy as np
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from config import NOISE_SEED
np.random.seed(NOISE_SEED)

# Now safe to import everything else
import argparse
from PIL import ImageFilter
import matplotlib
try:
    matplotlib.use("TkAgg")
except Exception:
    pass

from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from visualization.plots import setup_realtime_plot, plot_results
from performance_metrics import analyze_tracking_performance, print_performance_report
from config import *


def main():
    # ── CLI ───────────────────────────────────────────────────────────────────
    parser = argparse.ArgumentParser(description="Line-Following Robot Simulation")
    parser.add_argument(
        "--track", type=str, default="../assets/bane_fase2.png",
        help="Path to track image in assets/ (default: ../assets/bane_fase2.png)"
    )
    args = parser.parse_args()

    # ── Track ─────────────────────────────────────────────────────────────────
    track_path = os.path.normpath(os.path.join(os.path.dirname(__file__), args.track))
    print(f"Loading track: {track_path}")
    track    = load_track_image(track_path)
    blurred  = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    # ── Spawn ─────────────────────────────────────────────────────────────────
    track_filename = os.path.basename(track_path)
    W, H = MAP_SIZE_M
    if track_filename in SPAWN_REGISTRY:
        sp = SPAWN_REGISTRY[track_filename]
        x_start, y_start, theta_start = sp["x"], sp["y"], sp["theta"]
        print(f"Spawn: ({x_start}, {y_start})  theta={theta_start:.2f} rad")
    else:
        x_start, y_start, theta_start = 0.5 * W, 0.5 * H, 0.0
        print(f"No spawn entry for '{track_filename}' — spawning at centre.")
        print(f"  Tip: add it to SPAWN_REGISTRY in config.py for exact placement.")

    robot          = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta    = theta_start
    # Ensure wheel speeds start at zero for consistency
    robot.vL = 0.0
    robot.vR = 0.0
    sensors        = QTRArray()

    # ── Controllers (values from config.py) ───────────────────────────────────
    pid = PID(
        kp=PID_KP, ki=PID_KI, kd=PID_KD,
        limit=PID_LIMIT,
        integral_limit=PID_INTEGRAL_LIMIT,
        derivative_filter=PID_DERIV_FILTER,
    )
    # Reset PID state to ensure clean start
    pid.reset()

    speed_controller = SpeedController(
        straight_speed=SC_STRAIGHT_SPEED,
        turn_speed=SC_TURN_SPEED,
        error_threshold=SC_ERROR_THRESHOLD,
        smoothing=SC_SMOOTHING,
    )
    # Reset speed controller state to ensure clean start
    speed_controller.reset()

    # ── Checkpoint system ─────────────────────────────────────────────────────
    checkpoints = CHECKPOINT_REGISTRY.get(track_filename, [])
    cp_next     = 0
    cp_total    = len(checkpoints)
    if cp_total:
        print(f"Checkpoint system active: {cp_total} checkpoints must be cleared in order.")
    else:
        print("No checkpoints defined — lap valid via start/finish zone only.")

    # NOTE: Random seed is set at module import time (before matplotlib)
    # to ensure identical random state with optimizer

    # ── Live visualisation ────────────────────────────────────────────────────
    update_plot, _ = setup_realtime_plot(
        blur_arr,
        spawn=(x_start, y_start),
        sf_radius=START_FINISH_RADIUS,
        checkpoints=checkpoints,
    )

    traj, err_log, sensor_log = [], [], []
    last_e_y     = 0.0
    # LINE_THRESH is imported from config.py
    RENDER_EVERY = 8
    lap_started  = False
    lap_time     = None
    lap_start_t  = 0.0

    t           = 0.0
    step        = 0
    line_loss_t = 0.0

    while t < MAX_LAP_TIME:  # Use MAX_LAP_TIME (matches optimizer)
        # ── Sense ─────────────────────────────────────────────────────────────
        readings = sensors.read(robot, blur_arr)
        weights  = readings ** 2
        pos_y    = sensors.sensor_pos_body[:, 1]
        total_w  = weights.sum()

        if total_w > LINE_THRESH:
            e_y      = float(np.dot(weights, pos_y) / total_w)
            last_e_y = e_y
        else:
            e_y = last_e_y

        # ── Control ───────────────────────────────────────────────────────────
        w_cmd  = pid.compute(e_y, DT)
        v_cmd  = speed_controller.update(e_y, w_cmd, pid.limit)
        vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
        vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

        # ── Physics ───────────────────────────────────────────────────────────
        robot.update(vL_cmd, vR_cmd)
        traj.append(tuple(robot.position))
        err_log.append(e_y)
        sensor_log.append(readings.copy())

        rx, ry        = robot.position
        dist_to_start = np.hypot(rx - x_start, ry - y_start)

        # ── Checkpoints ───────────────────────────────────────────────────────
        if cp_next < cp_total:
            cx, cy = checkpoints[cp_next]
            if np.hypot(rx - cx, ry - cy) < CHECKPOINT_RADIUS:
                print(f"  Checkpoint {cp_next + 1}/{cp_total} cleared at t={t:.2f}s")
                cp_next += 1

        # ── Lap timer ─────────────────────────────────────────────────────────
        if not lap_started and dist_to_start > MIN_DEPARTURE_DIST:
            lap_started = True
            lap_start_t = t
        elif lap_started and lap_time is None and dist_to_start < START_FINISH_RADIUS:
            if cp_next < cp_total:
                print(f"  *** FALSE LAP — only {cp_next}/{cp_total} checkpoints cleared. "
                      f"Run invalid. ***")
                break
            else:
                lap_time = t - lap_start_t
                print(f"\n*** LAP COMPLETE — time: {lap_time:.3f} s ***\n")
                update_plot(robot, readings, e_y, robot.vL, robot.vR, t,
                            lap_time=lap_time, elapsed=lap_time,
                            cp_cleared=cp_next)
                break

        # ── Render ────────────────────────────────────────────────────────────
        if step % RENDER_EVERY == 0:
            elapsed = (t - lap_start_t) if lap_started and lap_time is None else lap_time
            update_plot(robot, readings, e_y, robot.vL, robot.vR, t,
                        lap_time=lap_time,
                        elapsed=elapsed if lap_started else None,
                        cp_cleared=cp_next)

        # ── Line-loss guard ───────────────────────────────────────────────────
        line_loss_t = line_loss_t + DT if total_w <= LINE_THRESH else 0.0
        if line_loss_t > MAX_LINE_LOSS_TIME:
            print("*** RUN INVALID — line lost for too long ***")
            break

        t    += DT
        step += 1

    # Keep the visualization window open until user closes it manually
    import matplotlib.pyplot as plt
    plt.show()


if __name__ == "__main__":
    main()
