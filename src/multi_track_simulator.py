# multi_track_simulator.py
"""
Run simulations on multiple tracks and aggregate results.
"""

import numpy as np
from PIL import ImageFilter
from track.multi_track import get_track_names, get_track
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from visualization.plots import setup_realtime_plot, plot_results
from performance_metrics import analyze_tracking_performance, print_performance_report
from config import *


def run_simulation_on_track(track_name, pid, speed_controller, show_visualization=False):
    """
    Run a single simulation on a specific track.

    Args:
        track_name: Name of the track from multi_track module
        pid: PID controller instance
        speed_controller: SpeedController instance
        show_visualization: Whether to show live visualization

    Returns:
        dict with metrics: {'track': name, 'max_error': ..., 'rms_error': ..., ...}
    """
    # Generate track
    track = get_track(track_name)
    blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    # Robot start position — per-track spawn registry
    W, H = MAP_SIZE_M
    SPAWN_REGISTRY = {
        "bane_fase2": {"x": 2.00, "y": 0.14, "theta": 0.0},
    }

    if track_name in SPAWN_REGISTRY:
        sp = SPAWN_REGISTRY[track_name]
        x_start, y_start, theta_start = sp["x"], sp["y"], sp["theta"]
    else:
        # Default: sine-wave start position
        x_start = 0.05 * W
        y_start = 0.5 * H + 0.25 * H * np.sin(3 * x_start / W * 2 * np.pi)
        omega = 3.0 * 2 * np.pi / W
        slope = 0.25 * H * omega * np.cos(omega * x_start)
        theta_start = np.arctan(slope)

    robot = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta = theta_start

    sensors = QTRArray()

    # Reset PID state
    pid.reset()

    # Setup visualization if requested
    if show_visualization:
        update_plot, robot_artist = setup_realtime_plot(blur_arr)
    else:
        update_plot = None

    traj, err_log, sensor_log = [], [], []
    last_e_y = 0.0
    LINE_THRESH = 0.08
    RENDER_EVERY = 8

    t = 0.0
    step = 0

    while t < SIM_TIME:
        # Read sensors
        readings = sensors.read(robot, blur_arr)

        # Compute lateral error
        weights = readings ** 2
        pos_y = sensors.sensor_pos_body[:, 1]
        total_w = weights.sum()

        if total_w > LINE_THRESH:
            e_y = float(np.dot(weights, pos_y) / total_w)
            last_e_y = e_y
        else:
            e_y = last_e_y

        # PID control
        w_cmd = pid.compute(e_y, DT)
        v_cmd = speed_controller.update(e_y, w_cmd, pid.limit)

        vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
        vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

        # Physics update
        robot.update(vL_cmd, vR_cmd)
        traj.append(tuple(robot.position))
        err_log.append(e_y)
        sensor_log.append(readings.copy())

        # Visualization
        if show_visualization and step % RENDER_EVERY == 0:
            update_plot(robot, readings, e_y, robot.vL, robot.vR, t)

        t += DT
        step += 1

    # Calculate metrics
    metrics = analyze_tracking_performance(err_log, DT)
    metrics['track'] = track_name

    return metrics, traj, err_log, sensor_log, blur_arr


def run_multi_track_test(pid, speed_controller, tracks=None, show_viz=False):
    """
    Run simulations on multiple tracks and return aggregated results.

    Args:
        pid: PID controller instance
        speed_controller: SpeedController instance
        tracks: List of track names to test. If None, tests all available.
        show_viz: Show visualization (only for first track)

    Returns:
        dict with aggregated statistics
    """
    if tracks is None:
        tracks = get_track_names()

    results = []
    all_metrics = []

    print("\n" + "="*70)
    print("MULTI-TRACK SIMULATION TEST")
    print("="*70)
    print(f"\nTesting PID: Kp={pid.kp}, Ki={pid.ki}, Kd={pid.kd}")
    print(f"Tracks to test: {tracks}\n")

    for i, track_name in enumerate(tracks):
        print(f"[{i+1}/{len(tracks)}] Running on '{track_name}' track...")

        try:
            metrics, traj, err_log, sensor_log, blur_arr = run_simulation_on_track(
                track_name, pid, speed_controller,
                show_visualization=(show_viz and i == 0)
            )
            results.append(metrics)
            all_metrics.append(metrics)

            print(f"      Max Error: {metrics['max_error']*1000:.2f}mm | "
                  f"RMS Error: {metrics['rms_error']*1000:.2f}mm | "
                  f"Settle Time: {metrics['settling_time']:.2f}s")
        except Exception as e:
            print(f"      ERROR: {e}")
            continue

    # Aggregate results
    if not results:
        print("No successful simulations!")
        return None

    print("\n" + "="*70)
    print("AGGREGATED RESULTS")
    print("="*70)

    aggregated = {
        'num_tracks': len(results),
        'avg_max_error': np.mean([r['max_error'] for r in results]),
        'avg_rms_error': np.mean([r['rms_error'] for r in results]),
        'avg_settling_time': np.mean([r['settling_time'] if r['settling_time'] else 0 for r in results]),
        'avg_steady_state_error': np.mean([r['steady_state_error'] for r in results]),
        'worst_max_error': max([r['max_error'] for r in results]),
        'worst_rms_error': max([r['rms_error'] for r in results]),
        'best_max_error': min([r['max_error'] for r in results]),
        'best_rms_error': min([r['rms_error'] for r in results]),
        'track_results': results
    }

    print(f"\nAverage Max Error:        {aggregated['avg_max_error']*1000:.2f}mm")
    print(f"Average RMS Error:        {aggregated['avg_rms_error']*1000:.2f}mm")
    print(f"Average Settling Time:    {aggregated['avg_settling_time']:.2f}s")
    print(f"Average Steady-State:     {aggregated['avg_steady_state_error']*1000:.2f}mm")
    print(f"\nWorst Max Error:          {aggregated['worst_max_error']*1000:.2f}mm ({results[max(range(len(results)), key=lambda i: results[i]['max_error'])]['track']})")
    print(f"Best Max Error:           {aggregated['best_max_error']*1000:.2f}mm ({results[min(range(len(results)), key=lambda i: results[i]['max_error'])]['track']})")
    print("="*70 + "\n")

    return aggregated

