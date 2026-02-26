# visualization/plots.py
import matplotlib
import os
if 'MPLBACKEND' not in os.environ:
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass

import matplotlib.pyplot as plt
import numpy as np
from config import MAP_SIZE_M, QTR_CHANNELS, QTR_SPACING_M, QTR_SENSOR_OFFSET_M, DT, CHECKPOINT_RADIUS

BG         = "#0d1117"
PANEL_BG   = "#161b22"
BORDER     = "#30363d"
PATH_COL   = "#58a6ff"
ROBOT_COL  = "#f78166"
SENSOR_ON  = "#39d353"
SENSOR_OFF = "#21262d"
ERR_COL    = "#d2a8ff"
VL_COL     = "#58a6ff"
VR_COL     = "#ffa657"
ZERO_COL   = "#484f58"
CP_PENDING = "#ffd700"
CP_DONE    = "#39d353"


def _style_ax(ax, title):
    ax.set_facecolor(PANEL_BG)
    ax.set_title(title, color="white", fontsize=10, pad=6)
    ax.tick_params(colors="#8b949e", labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER)


def setup_realtime_plot(map_arr, spawn=None, sf_radius=0.10, checkpoints=None):
    """
    Create the live dashboard. Returns (update_fn, robot_artist).
    Call update_fn(robot, readings, e_y, vL, vR, t, lap_time, elapsed, cp_cleared)
    each simulation step.
    """
    W, H = MAP_SIZE_M
    n = QTR_CHANNELS

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(15, 8), facecolor=BG)
    fig.canvas.manager.set_window_title("Line-Following Robot – Live Simulation")

    gs = fig.add_gridspec(3, 2, width_ratios=[2.2, 1],
                          hspace=0.5, wspace=0.28,
                          left=0.05, right=0.97, top=0.94, bottom=0.07)

    ax_map    = fig.add_subplot(gs[:, 0])
    ax_sensor = fig.add_subplot(gs[0, 1])
    ax_err    = fig.add_subplot(gs[1, 1])
    ax_speed  = fig.add_subplot(gs[2, 1])

    ax_map.imshow(map_arr, cmap="gray", vmin=0, vmax=255,
                  extent=[0, W, 0, H], interpolation="bilinear", aspect="equal")
    ax_map.set_facecolor(BG)
    ax_map.set_title("Robot Live View", color="white", fontsize=13, pad=8)
    ax_map.set_xlabel("x  (m)", color="#8b949e", fontsize=9)
    ax_map.set_ylabel("y  (m)", color="#8b949e", fontsize=9)
    ax_map.tick_params(colors="#8b949e")
    for sp in ax_map.spines.values():
        sp.set_edgecolor(BORDER)

    path_line,   = ax_map.plot([], [], color=PATH_COL, lw=1.8, alpha=0.9, zorder=3)
    robot_circle = plt.Circle((0, 0), radius=0.025, color=ROBOT_COL, zorder=7, alpha=0.9)
    ax_map.add_patch(robot_circle)
    heading_line, = ax_map.plot([], [], color="white", lw=2, zorder=8)
    sensor_scat   = ax_map.scatter(np.zeros(n), np.zeros(n),
                                   c=["gray"] * n, s=16, zorder=6, linewidths=0)

    if spawn is not None:
        sx, sy = spawn
        ax_map.add_patch(plt.Circle((sx, sy), sf_radius,
                                    color="#00ff88", alpha=0.18, zorder=2))
        ax_map.add_patch(plt.Circle((sx, sy), sf_radius,
                                    fill=False, edgecolor="#00ff88",
                                    linewidth=2.0, linestyle="--", zorder=4))
        ax_map.plot(sx, sy, "o", color="#00ff88", markersize=6, zorder=5)

    _checkpoints = checkpoints or []
    cp_fills, cp_rings, cp_labels = [], [], []
    for i, (cx, cy) in enumerate(_checkpoints):
        fill = plt.Circle((cx, cy), CHECKPOINT_RADIUS,
                           color=CP_PENDING, alpha=0.15, zorder=2)
        ring = plt.Circle((cx, cy), CHECKPOINT_RADIUS,
                           fill=False, edgecolor=CP_PENDING,
                           linewidth=2.0, linestyle="-", zorder=4)
        ax_map.add_patch(fill)
        ax_map.add_patch(ring)
        lbl = ax_map.text(cx, cy, str(i + 1), color=CP_PENDING,
                          fontsize=9, fontweight="bold",
                          ha="center", va="center", zorder=5)
        cp_fills.append(fill); cp_rings.append(ring); cp_labels.append(lbl)

    timer_text = ax_map.text(
        0.02, 0.97, "", transform=ax_map.transAxes,
        fontsize=11, fontweight="bold", color="#00ff88",
        va="top", ha="left", zorder=10,
        bbox=dict(boxstyle="round,pad=0.3", facecolor=BG,
                  edgecolor="#00ff88", alpha=0.85))

    _style_ax(ax_sensor, "Sensor Array")
    bar_xs = (np.arange(n) - (n - 1) / 2) * QTR_SPACING_M * 100   # cm
    bars = ax_sensor.bar(bar_xs, np.zeros(n), width=QTR_SPACING_M * 85,
                         color=SENSOR_OFF, edgecolor="none")
    ax_sensor.set_ylim(0, 1.1)
    ax_sensor.set_xlim(bar_xs[0] - 0.15, bar_xs[-1] + 0.15)
    ax_sensor.set_xlabel("lateral position (cm)", color="#8b949e", fontsize=8)
    ax_sensor.set_ylabel("reading", color="#8b949e", fontsize=8)
    centroid_vline = ax_sensor.axvline(0, color=ROBOT_COL, lw=1.5, ls="--", alpha=0.85, zorder=5)
    ax_sensor.axhline(0.15, color=ZERO_COL, lw=0.8, ls=":")

    _style_ax(ax_err, "Lateral Error  e_y")
    err_line, = ax_err.plot([], [], color=ERR_COL, lw=1.4)
    ax_err.axhline(0, color=ZERO_COL, lw=0.8)
    ax_err.set_xlabel("time (s)", color="#8b949e", fontsize=8)
    ax_err.set_ylabel("m", color="#8b949e", fontsize=8)

    _style_ax(ax_speed, "Wheel Speeds")
    spd_L, = ax_speed.plot([], [], color=VL_COL, lw=1.3, label="vL")
    spd_R, = ax_speed.plot([], [], color=VR_COL, lw=1.3, label="vR")
    ax_speed.axhline(0, color=ZERO_COL, lw=0.8)
    ax_speed.set_xlabel("time (s)", color="#8b949e", fontsize=8)
    ax_speed.set_ylabel("m/s", color="#8b949e", fontsize=8)
    ax_speed.legend(fontsize=7, facecolor=PANEL_BG, edgecolor=BORDER,
                    labelcolor="white", loc="upper right")

    plt.show(block=False)
    fig.canvas.draw()

    traj_x, traj_y = [], []
    t_log, err_buf, vL_buf, vR_buf = [], [], [], []
    ARROW = QTR_SENSOR_OFFSET_M

    def update(robot, readings, e_y, vL, vR, t,
               lap_time=None, elapsed=None, cp_cleared=0):
        px, py = float(robot.position[0]), float(robot.position[1])
        traj_x.append(px); traj_y.append(py)
        t_log.append(t)
        err_buf.append(e_y)
        vL_buf.append(vL); vR_buf.append(vR)

        for i, (fill, ring, lbl) in enumerate(zip(cp_fills, cp_rings, cp_labels)):
            col = CP_DONE if i < cp_cleared else CP_PENDING
            fill.set_facecolor(col); ring.set_edgecolor(col); lbl.set_color(col)

        if lap_time is not None:
            timer_text.set_text(f"LAP: {lap_time:.3f} s")
            timer_text.set_color("#ffd700")
            timer_text.get_bbox_patch().set_edgecolor("#ffd700")
        elif elapsed is not None:
            timer_text.set_text(f"TIME: {elapsed:.2f} s")
        else:
            timer_text.set_text("TIME: --")

        path_line.set_data(traj_x, traj_y)
        robot_circle.set_center((px, py))
        hdx = ARROW * np.cos(robot.theta)
        hdy = ARROW * np.sin(robot.theta)
        heading_line.set_data([px, px + hdx], [py, py + hdy])

        c, s = np.cos(robot.theta), np.sin(robot.theta)
        R = np.array([[c, -s], [s, c]])
        body_pts = np.column_stack([
            np.full(n, QTR_SENSOR_OFFSET_M),
            (np.arange(n) - (n - 1) / 2) * QTR_SPACING_M])
        world_pts = (R @ body_pts.T).T + robot.position
        colors = [SENSOR_ON if r > 0.15 else SENSOR_OFF for r in readings]
        sensor_scat.set_offsets(world_pts)
        sensor_scat.set_color(colors)

        for bar, val, col in zip(bars, readings, colors):
            bar.set_height(float(val)); bar.set_color(col)
        weights = readings ** 2
        w_sum   = weights.sum()
        centroid_vline.set_xdata([float(np.dot(weights, bar_xs) / w_sum) if w_sum > 0.05 else 0.0])

        tarr = np.array(t_log)
        err_line.set_data(tarr, err_buf)
        t_lo = max(0.0, t - 8.0); t_hi = max(8.0, t + 0.3)
        ax_err.set_xlim(t_lo, t_hi)
        recent = err_buf[-400:] if err_buf else [0]
        emax = max(0.04, max(abs(v) for v in recent) * 1.3)
        ax_err.set_ylim(-emax, emax)

        spd_L.set_data(tarr, vL_buf)
        spd_R.set_data(tarr, vR_buf)
        ax_speed.set_xlim(t_lo, t_hi)
        ax_speed.set_ylim(-0.2, 1.5)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    return update, (robot_circle, heading_line)


def plot_results(traj, sensor_log, err_log, map_arr=None):
    """Static summary shown after the simulation ends."""
    W, H = MAP_SIZE_M
    x = [p[0] for p in traj]
    y = [p[1] for p in traj]
    t = np.arange(len(err_log)) * DT

    plt.style.use("dark_background")
    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(14, 6), facecolor=BG)
    fig.suptitle("Simulation Summary", color="white", fontsize=14, y=0.97)

    if map_arr is not None:
        ax_map.imshow(map_arr, cmap="gray", vmin=0, vmax=255,
                      extent=[0, W, 0, H], interpolation="bilinear", aspect="equal")
    sc = ax_map.scatter(x, y, c=np.linspace(0, 1, len(x)),
                        cmap="plasma", s=5, zorder=3, alpha=0.8)
    cbar = plt.colorbar(sc, ax=ax_map, shrink=0.75, pad=0.02)
    cbar.set_label("time →", color="#8b949e", fontsize=8)
    cbar.ax.yaxis.set_tick_params(color="#8b949e", labelsize=7)

    _style_ax(ax_map, "Final Trajectory")
    ax_map.set_xlabel("x (m)", color="#8b949e")
    ax_map.set_ylabel("y (m)", color="#8b949e")

    ax_err.plot(t, err_log, color=ERR_COL, lw=1.2)
    ax_err.axhline(0, color=ZERO_COL, lw=0.8)
    ax_err.fill_between(t, err_log, 0, alpha=0.15, color=ERR_COL)
    _style_ax(ax_err, "Lateral Error Over Time")
    ax_err.set_xlabel("time (s)", color="#8b949e")
    ax_err.set_ylabel("e_y (m)", color="#8b949e")

    plt.tight_layout()
    plt.show()

