# main.py

import argparse
import os
from PIL import ImageFilter
import matplotlib
try:
    matplotlib.use("TkAgg")
except Exception:
    pass  # Fall back to system default (works on Windows, Linux, macOS)

from track.track_generator import generate_default_track
from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from control.pid_controller import PID, SpeedController
from visualization.plots import setup_realtime_plot, plot_results
from performance_metrics import analyze_tracking_performance, print_performance_report
from config import *

# START_FINISH_RADIUS, MIN_DEPARTURE_DIST, MAX_LAP_TIME, NOISE_SEED
# are all defined in config.py — edit there to affect both main.py and lap_optimizer.py


def main():
    # ── CLI arguments ─────────────────────────────────────────────────────────
    parser = argparse.ArgumentParser(description="Line-Following Robot Simulation")
    parser.add_argument(
        "--track",
        type=str,
        default=None,
        help="Path to a custom track image (e.g. ../assets/suzuka.png). "
             "If omitted, the default generated sine-wave track is used."
    )
    args = parser.parse_args()

    # ── Track ────────────────────────────────────────────────────────────────
    # Ensure track_path is defined before use
    if args.track:
        track_path = os.path.join(os.path.dirname(__file__), args.track)
        track_path = os.path.normpath(track_path)
    else:
        track_path = None  # Default to None if no track is provided

    if track_path:
        print(f"Loading custom track: {track_path}")
        track = load_track_image(track_path)
    else:
        track = generate_default_track()

    blurred  = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    # ── Robot start position ──────────────────────────────────────────────────
    # Spawn positions are defined in config.py → SPAWN_REGISTRY
    W, H = MAP_SIZE_M

    if track_path:
        track_filename = os.path.basename(track_path)
        if track_filename in SPAWN_REGISTRY:
            sp = SPAWN_REGISTRY[track_filename]
            x_start, y_start, theta_start = sp["x"], sp["y"], sp["theta"]
            print(f"Using hardcoded spawn for '{track_filename}': "
                  f"({x_start}, {y_start}), theta={theta_start:.2f} rad")
        else:
            # Unknown track: spawn at centre and search
            x_start, y_start, theta_start = 0.5 * W, 0.5 * H, 0.0
            print(f"No spawn entry for '{track_filename}' — "
                  f"spawning at centre and searching for line.")
            print(f"  Tip: add it to SPAWN_REGISTRY in main.py for exact placement.")
    else:
        # Generated sine-wave track: spawn exactly on the line
        x_start = 0.05 * W
        y_start = 0.5 * H + 0.25 * H * np.sin(3 * x_start / W * 2 * np.pi)
        omega       = 3.0 * 2 * np.pi / W
        slope       = 0.25 * H * omega * np.cos(omega * x_start)
        theta_start = np.arctan(slope)

    robot          = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta    = theta_start

    sensors = QTRArray()

    # ── PID ───────────────────────────────────────────────────────────────────
    # e_y > 0 → line is to robot's LEFT  → steer LEFT  → positive ω
    # Differential drive:  w = (vR-vL)/WB,  vL = v - w·WB/2,  vR = v + w·WB/2
    # So pid output directly becomes angular velocity command.
    # Aggressive tuning: fast turns, strong recovery, minimal line loss
    pid = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD,
              limit=PID_LIMIT,
              integral_limit=PID_INTEGRAL_LIMIT,
              derivative_filter=PID_DERIV_FILTER)

    # ── Speed Controller (State Machine) ──────────────────────────────────────
    # Increases speed when robot is on straight sections (low error, low turning)
    # Reduces speed during turns for stability
    speed_controller = SpeedController(
        straight_speed=SC_STRAIGHT_SPEED,
        turn_speed=SC_TURN_SPEED,
        error_threshold=SC_ERROR_THRESHOLD,
        smoothing=SC_SMOOTHING,
    )

    # ── Checkpoint system ─────────────────────────────────────────────────────
    _track_fn   = os.path.basename(track_path) if track_path else None
    checkpoints = CHECKPOINT_REGISTRY.get(_track_fn, []) if _track_fn else []
    cp_next     = 0          # index of the next checkpoint to reach
    cp_total    = len(checkpoints)
    if cp_total:
        print(f"Checkpoint system active: {cp_total} checkpoints must be cleared in order.")
    else:
        print("No checkpoints defined for this track — lap validation via start/finish zone only.")

    # ── Live visualisation ────────────────────────────────────────────────────
    update_plot, robot_artist = setup_realtime_plot(
        blur_arr,
        spawn=(x_start, y_start),
        sf_radius=START_FINISH_RADIUS,
        checkpoints=checkpoints,
    )

    traj, err_log, sensor_log = [], [], []
    last_e_y     = 0.0
    LINE_THRESH  = 0.08
    RENDER_EVERY = 8

    # ── Lap timer state ───────────────────────────────────────────────────────
    lap_started = False
    lap_time    = None
    lap_start_t = 0.0


    np.random.seed(NOISE_SEED)   # deterministic sensor noise — matches optimizer
    t           = 0.0
    step        = 0
    line_loss_t = 0.0

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

        # ── Lap timer ─────────────────────────────────────────────────────────
        dist_to_start = np.hypot(robot.position[0] - x_start,
                                 robot.position[1] - y_start)

        # ── Advance checkpoint counter ─────────────────────────────────────────
        if cp_next < cp_total:
            cx, cy = checkpoints[cp_next]
            if np.hypot(robot.position[0] - cx, robot.position[1] - cy) < CHECKPOINT_RADIUS:
                print(f"  Checkpoint {cp_next + 1}/{cp_total} cleared at t={t:.2f}s")
                cp_next += 1

        if not lap_started and dist_to_start > MIN_DEPARTURE_DIST:
            lap_started = True
            lap_start_t = t
        if lap_started and lap_time is None and dist_to_start < START_FINISH_RADIUS:
            if cp_next < cp_total:
                print(f"  *** FALSE LAP — only {cp_next}/{cp_total} checkpoints cleared. "
                      f"Continuing... ***")
            else:
                lap_time = t - lap_start_t
                print(f"\n*** LAP COMPLETE — time: {lap_time:.3f} s ***\n")
                # Render the final frame then stop immediately
                update_plot(robot, readings, e_y, robot.vL, robot.vR, t,
                            lap_time=lap_time, elapsed=lap_time,
                            cp_cleared=cp_next)
                break

        # ── Render ────────────────────────────────────────────────────────────
        if step % RENDER_EVERY == 0:
            elapsed = (t - lap_start_t) if lap_started and lap_time is None else lap_time
            update_plot(robot, readings, e_y, robot.vL, robot.vR, t,
                        lap_time=lap_time, elapsed=elapsed if lap_started else None,
                        cp_cleared=cp_next)

        # Handle line loss termination
        if total_w <= LINE_THRESH:
            line_loss_t += DT
        else:
            line_loss_t = 0.0

        if line_loss_t > MAX_LINE_LOSS_TIME:
            print("*** RUN INVALID — line lost for too long ***")
            break

        t    += DT
        step += 1

    # ── Summary ───────────────────────────────────────────────────────────────
    metrics = analyze_tracking_performance(err_log, DT)
    print_performance_report(metrics)

    if lap_time is not None:
        print(f"Lap time: {lap_time:.3f} s")
    else:
        print("No lap completed (robot did not return to start/finish zone)")

    plot_results(traj, sensor_log, err_log,
                 map_arr=np.array(blurred, dtype=np.uint8))


if __name__ == "__main__":
    main()
