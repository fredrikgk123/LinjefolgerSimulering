# simulator.py — robot physics, FSM/PID, sensor model, episode runner,
#                dashboard, headless runner, and tuner.

import os
import sys
import time as _time
from enum import Enum, auto

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageFilter

import config as C_
import physics as P

# ── Console helpers ───────────────────────────────────────────────────────────

_TTY = sys.stdout.isatty()

class C:
    RESET  = "\033[0m"   if _TTY else ""
    BOLD   = "\033[1m"   if _TTY else ""
    DIM    = "\033[2m"   if _TTY else ""
    GREEN  = "\033[92m"  if _TTY else ""
    YELLOW = "\033[93m"  if _TTY else ""
    RED    = "\033[91m"  if _TTY else ""
    CYAN   = "\033[96m"  if _TTY else ""
    WHITE  = "\033[97m"  if _TTY else ""
    ERASE  = "\033[2K\r" if _TTY else "\r"

W = 68

def _rule():    print(C.DIM + "─" * W + C.RESET)
def _header(t): print(C.BOLD+C.CYAN + "═"*W + f"\n{' '*((W-len(t)-2)//2)} {t} \n" + "═"*W + C.RESET)
def _section(t):print(C.DIM+"─"*W+C.RESET+"\n"+C.BOLD+f"  {t}"+C.RESET)
def _kv(k,v,col=None): print(f"  {C.DIM}{k:<18}{C.RESET}{col or C.WHITE}{v}{C.RESET}")
def _ok(m):   print(f"  {C.GREEN}✔{C.RESET}  {m}")
def _warn(m): print(f"  {C.YELLOW}⚠{C.RESET}  {m}")
def _err(m):  print(f"  {C.RED}✗{C.RESET}  {m}")

# ── Track loader ──────────────────────────────────────────────────────────────

def load_track(path: str) -> np.ndarray:
    img = Image.open(path).convert("L")
    arr = np.array(img)
    if arr.mean() < 128:
        arr = 255 - arr
        img = Image.fromarray(arr)
    img = img.resize(
        (int(C_.MAP_W_M * C_.PX_PER_M), int(C_.MAP_H_M * C_.PX_PER_M)),
        resample=getattr(Image, "Resampling", Image).LANCZOS)
    return np.array(img.filter(ImageFilter.GaussianBlur(radius=2)), dtype=np.float32)

# ── Coordinate helpers ────────────────────────────────────────────────────────

def _world_to_px(x, y):
    px = np.clip(x * C_.PX_PER_M, 0, int(C_.MAP_W_M * C_.PX_PER_M) - 1)
    py = np.clip((C_.MAP_H_M - y) * C_.PX_PER_M, 0, int(C_.MAP_H_M * C_.PX_PER_M) - 1)
    return px, py

def _bilinear(arr, x, y):
    x0, y0 = int(x), int(y)
    x1 = min(x0 + 1, arr.shape[1] - 1)
    y1 = min(y0 + 1, arr.shape[0] - 1)
    dx, dy = x - x0, y - y0
    return ((1-dx)*(1-dy)*arr[y0,x0] + dx*(1-dy)*arr[y0,x1]
            + (1-dx)*dy*arr[y1,x0] + dx*dy*arr[y1,x1])

# ── FSM + PID ─────────────────────────────────────────────────────────────────

class State(Enum):
    STRAIGHT = auto()
    CORNER   = auto()
    SHARP    = auto()
    LOST     = auto()

STATE_COLOURS = {
    State.STRAIGHT: "#39d353",
    State.CORNER:   "#ffa657",
    State.SHARP:    "#f78166",
    State.LOST:     "#8b949e",
}

class FSM:
    _GAINS = {
        State.STRAIGHT: None,
        State.CORNER:   None,
        State.SHARP:    None,
        State.LOST:     None,
    }

    def __init__(self, gains=None, fsm_params=None):
        g = gains or {
            State.STRAIGHT: C_.PID_STRAIGHT,
            State.CORNER:   C_.PID_CORNER,
            State.SHARP:    C_.PID_SHARP,
            State.LOST:     C_.PID_LOST,
        }
        fp = fsm_params or {}
        self._gains       = g
        self._str_exit    = fp.get("fsm_str_exit",    C_.FSM_STR_EXIT)
        self._str_enter   = fp.get("fsm_str_enter",   C_.FSM_STR_ENTER)
        self._shp_enter   = fp.get("fsm_sharp_enter", C_.FSM_SHARP_ENTER)
        self._shp_exit    = fp.get("fsm_sharp_exit",  C_.FSM_SHARP_EXIT)
        self._lost_w      = fp.get("fsm_lost_search_w", C_.FSM_LOST_SEARCH_W)
        self._deriv_alpha = fp.get("deriv_alpha",     C_.DERIV_ALPHA)
        self._accel_rate  = fp.get("accel_rate",      C_.ACCEL_RATE)

        self.state       = State.STRAIGHT
        self.integral    = 0.0
        self.prev_err    = 0.0
        self.d_filt      = 0.0
        self.v_filt      = 0.0
        self.last_e_sign = 0.0

    def _next_state(self, ae, tw):
        if tw < C_.FSM_LOST_WEIGHT:
            return State.LOST
        if self.state == State.LOST:
            return State.CORNER if tw >= C_.FSM_FOUND_WEIGHT else State.LOST
        if self.state == State.STRAIGHT:
            if ae > self._shp_enter: return State.SHARP
            if ae > self._str_exit:  return State.CORNER
            return State.STRAIGHT
        if self.state == State.CORNER:
            if ae > self._shp_enter: return State.SHARP
            if ae < self._str_enter: return State.STRAIGHT
            return State.CORNER
        if self.state == State.SHARP:
            return State.CORNER if ae < self._shp_exit else State.SHARP
        return self.state

    def update(self, e_norm, total_weight, dt):
        ae        = abs(e_norm)
        new_state = self._next_state(ae, total_weight)
        if new_state != self.state:
            self.integral = 0.0
            self.d_filt   = 0.0
            self.prev_err = e_norm
            self.state    = new_state
        if total_weight >= C_.FSM_FOUND_WEIGHT and e_norm != 0.0:
            self.last_e_sign = 1.0 if e_norm > 0 else -1.0

        kp, ki, kd, limit, target_v = self._gains[self.state]

        if self.state == State.LOST:
            w_cmd = self.last_e_sign * self._lost_w
        else:
            p             = kp * e_norm
            self.integral = float(np.clip(self.integral + e_norm * dt, -C_.INTEG_LIMIT, C_.INTEG_LIMIT))
            d_raw         = (e_norm - self.prev_err) / dt
            self.d_filt   = self._deriv_alpha * d_raw + (1 - self._deriv_alpha) * self.d_filt
            w_cmd         = float(np.clip(p + ki * self.integral + kd * self.d_filt, -limit, limit))
            self.prev_err = e_norm

        self.v_filt = (target_v if self.v_filt > target_v
                       else min(target_v, self.v_filt + self._accel_rate * dt))
        return w_cmd, self.v_filt, self.state

# ── Robot physics ─────────────────────────────────────────────────────────────

class Robot:
    def __init__(self, x, y, theta):
        self.pos   = np.array([x, y], dtype=float)
        self.theta = float(theta)
        self.vL = self.vR = self.omega = self.v = 0.0

    @staticmethod
    def _motor_accel(v_wheel, v_cmd):
        v_drive  = (v_cmd / P.MAX_WHEEL) * P.MOTOR_V
        back_emf = P.MOTOR_KE * (v_wheel / P.WHEEL_RADIUS)
        current  = np.clip(v_drive - back_emf, -P.MOTOR_V, P.MOTOR_V) / P.MOTOR_R
        accel    = (P.MOTOR_KT * current / P.WHEEL_RADIUS) / (P.ROBOT_MASS / 2.0)
        return float(np.clip(accel, -800.0, 800.0))

    def update(self, vL_cmd, vR_cmd):
        if abs(vL_cmd) < P.DEADZONE: vL_cmd = 0.0
        if abs(vR_cmd) < P.DEADZONE: vR_cmd = 0.0

        self.vL = float(np.clip(self.vL + self._motor_accel(self.vL, vL_cmd) * C_.DT, -P.MAX_WHEEL, P.MAX_WHEEL))
        self.vR = float(np.clip(self.vR + self._motor_accel(self.vR, vR_cmd) * C_.DT, -P.MAX_WHEEL, P.MAX_WHEEL))

        v_demand     = 0.5 * (self.vL + self.vR)
        omega_demand = (self.vR - self.vL) / P.WHEEL_BASE

        a_fwd            = (v_demand - self.v) / C_.DT
        delta_load       = -(P.ROBOT_MASS * a_fwd * P.COM_HEIGHT) / (P._AXLE_TO_FRONT * P.ROBOT_MASS * P.G)
        drive_load       = float(np.clip(P.DRIVE_LOAD_REST + delta_load, 0.10, 1.0))
        F_normal         = drive_load * P.ROBOT_MASS * P.G

        TAU_YAW = float(np.clip(P.ROBOT_IZ / (P.ROBOT_MASS * (P.WHEEL_BASE / 2.0)**2), P.MOTOR_TAU, 0.40))
        self.omega += (omega_demand - self.omega) * (C_.DT / TAU_YAW)

        if abs(self.omega) > 1e-6:
            v_demand = float(np.clip(v_demand,
                                     -(P.MU_LATERAL * F_normal) / (P.ROBOT_MASS * abs(self.omega)),
                                      (P.MU_LATERAL * F_normal) / (P.ROBOT_MASS * abs(self.omega))))
        self.v = v_demand
        self.pos   += np.array([np.cos(self.theta), np.sin(self.theta)]) * self.v * C_.DT
        self.theta += self.omega * C_.DT

    def sensor_world_pos(self):
        c, s = np.cos(self.theta), np.sin(self.theta)
        R    = np.array([[c, -s], [s, c]])
        ys   = (np.arange(P.N_SENSORS) - (P.N_SENSORS - 1) / 2) * P.SPACING_M
        body = np.stack([np.full(P.N_SENSORS, P.SENSOR_FWD), ys], axis=1)
        return (R @ body.T).T + self.pos

# ── Sensor reading ────────────────────────────────────────────────────────────

_rng = np.random.default_rng(42)

def read_sensors(robot: Robot, track: np.ndarray) -> np.ndarray:
    readings = []
    for wx, wy in robot.sensor_world_pos():
        px, py = _world_to_px(wx, wy)
        readings.append(1.0 - _bilinear(track, px, py) / 255.0)
    arr = np.clip(np.array(readings) + _rng.standard_normal(P.N_SENSORS) * P.NOISE_STD, 0, 1)
    return np.round(arr * 1000) / 1000

# ── Dashboard ─────────────────────────────────────────────────────────────────

def build_dashboard(track_arr, spawn_xy, checkpoints=None):
    BG, PANEL      = "#0d1117", "#161b22"
    CP_PENDING     = "#f0e040"
    CP_HIT         = "#39d353"
    CP_MISSED      = "#f78166"

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 7), facecolor=BG)
    fig.canvas.manager.set_window_title("Line-Following Robot Simulator")

    gs     = fig.add_gridspec(3, 2, width_ratios=[2.2, 1], hspace=0.55, wspace=0.30,
                               left=0.05, right=0.97, top=0.93, bottom=0.07)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_sen = fig.add_subplot(gs[0, 1])
    ax_err = fig.add_subplot(gs[1, 1])
    ax_spd = fig.add_subplot(gs[2, 1])

    ax_map.imshow(track_arr, cmap="gray", vmin=0, vmax=255,
                  extent=(0, C_.MAP_W_M, 0, C_.MAP_H_M), interpolation="bilinear", aspect="equal")
    ax_map.set_facecolor(BG)
    ax_map.set_title("Robot Live View", color="white", fontsize=13)
    ax_map.set_xlabel("x (m)", color="#8b949e"); ax_map.set_ylabel("y (m)", color="#8b949e")
    ax_map.tick_params(colors="#8b949e")

    sx, sy = spawn_xy
    ax_map.add_patch(plt.Circle((sx, sy), 0.10, color="#00ff88", alpha=0.18, zorder=2))
    ax_map.add_patch(plt.Circle((sx, sy), 0.10, color="#00ff88", alpha=0.55, fill=False, lw=1.5, zorder=2))
    ax_map.text(sx, sy + 0.13, "S/F", color="#00ff88", fontsize=7, ha="center", va="bottom", zorder=2)

    cp_circles, cp_rings = [], []
    if checkpoints:
        for i, (cx, cy, cr) in enumerate(checkpoints):
            ring = plt.Circle((cx, cy), cr, color=CP_PENDING, fill=False, lw=1.2, linestyle="--", alpha=0.70, zorder=4)
            fill = plt.Circle((cx, cy), cr * 0.45, color=CP_PENDING, alpha=0.25, zorder=4)
            ax_map.add_patch(ring); ax_map.add_patch(fill)
            ax_map.text(cx, cy, f"CP{i+1}", color=CP_PENDING, fontsize=7, ha="center", va="center", fontweight="bold", zorder=5)
            cp_rings.append(ring); cp_circles.append(fill)

    path_line,    = ax_map.plot([], [], color="#58a6ff", lw=1.8, alpha=0.9, zorder=3)
    robot_dot      = plt.Circle((sx, sy), P.WHEEL_BASE / 2, color="#f78166", zorder=7)
    heading_line, = ax_map.plot([], [], color="white", lw=2, zorder=8)
    sensor_scat    = ax_map.scatter(np.zeros(P.N_SENSORS), np.zeros(P.N_SENSORS), c=["#21262d"]*P.N_SENSORS, s=18, zorder=6)
    ax_map.add_patch(robot_dot)

    ax_sen.set_facecolor(PANEL); ax_sen.set_title("Sensor readings", color="white", fontsize=9)
    ax_sen.set_ylim(-0.05, 1.1); ax_sen.set_xlim(-0.5, P.N_SENSORS - 0.5)
    ax_sen.tick_params(colors="#8b949e", labelsize=7)
    sensor_bars = ax_sen.bar(range(P.N_SENSORS), [0]*P.N_SENSORS, color="#21262d", width=0.8)

    ax_err.set_facecolor(PANEL); ax_err.set_title("Lateral error (m)", color="white", fontsize=9)
    ax_err.set_ylim(-P.HALF_SPAN * 1.3, P.HALF_SPAN * 1.3)
    ax_err.axhline(0, color="#484f58", lw=0.8); ax_err.tick_params(colors="#8b949e", labelsize=7)
    err_line, = ax_err.plot([], [], color="#d2a8ff", lw=1.2)

    ax_spd.set_facecolor(PANEL); ax_spd.set_title("Wheel speeds (m/s)", color="white", fontsize=9)
    ax_spd.set_ylim(-0.2, P.MAX_WHEEL + 0.2)
    ax_spd.axhline(0, color="#484f58", lw=0.8); ax_spd.tick_params(colors="#8b949e", labelsize=7)
    vl_line, = ax_spd.plot([], [], color="#58a6ff", lw=1.2, label="Left")
    vr_line, = ax_spd.plot([], [], color="#ffa657", lw=1.2, label="Right")
    ax_spd.legend(fontsize=7, loc="upper right", facecolor=PANEL, labelcolor="white", edgecolor="#30363d")

    time_txt  = ax_map.text(0.02, 0.97, "t = 0.00 s", transform=ax_map.transAxes,
                             color="white", fontsize=11, va="top",
                             bbox=dict(boxstyle="round,pad=0.3", facecolor=PANEL, alpha=0.7))
    state_txt = ax_map.text(0.98, 0.97, "STRAIGHT", transform=ax_map.transAxes,
                             color=STATE_COLOURS[State.STRAIGHT], fontsize=11,
                             va="top", ha="right", fontweight="bold",
                             bbox=dict(boxstyle="round,pad=0.3", facecolor=PANEL, alpha=0.7))
    plt.ion(); plt.show()

    xs, ys_t, t_hist, e_hist, vl_hist, vr_hist = [], [], [], [], [], []
    WINDOW = int(5.0 / C_.DT)

    def update(robot, readings, e_y, t, lap_count=0, last_lap=None,
               fsm_state=State.STRAIGHT, cp_hit_mask=None, lap_invalid=False):
        xs.append(robot.pos[0]); ys_t.append(robot.pos[1])
        path_line.set_data(xs, ys_t)

        rx, ry = robot.pos
        sc = STATE_COLOURS[fsm_state]
        robot_dot.center = (rx, ry); robot_dot.set_color(sc)
        heading_line.set_data([rx, rx + 0.05*np.cos(robot.theta)],
                               [ry, ry + 0.05*np.sin(robot.theta)])

        sensor_scat.set_offsets(robot.sensor_world_pos())
        cols = [sc if r > 0.3 else "#21262d" for r in readings]
        sensor_scat.set_color(cols)
        for bar, r in zip(sensor_bars, readings):
            bar.set_height(r); bar.set_color(sc if r > 0.3 else "#21262d")

        if cp_hit_mask is not None:
            for i, hit in enumerate(cp_hit_mask):
                col = CP_MISSED if lap_invalid else (CP_HIT if hit else CP_PENDING)
                cp_circles[i].set_facecolor(col)
                cp_circles[i].set_alpha(0.45 if (hit or lap_invalid) else 0.25)
                cp_rings[i].set_edgecolor(col)

        t_hist.append(t); e_hist.append(e_y)
        err_line.set_data(t_hist[-WINDOW:], e_hist[-WINDOW:])
        ax_err.set_xlim(max(0, t - 5), t + 0.1)

        vl_hist.append(robot.vL); vr_hist.append(robot.vR)
        vl_line.set_data(t_hist[-WINDOW:], vl_hist[-WINDOW:])
        vr_line.set_data(t_hist[-WINDOW:], vr_hist[-WINDOW:])
        ax_spd.set_xlim(max(0, t - 5), t + 0.1)

        lap_str = (f"  |  Lap {lap_count}  —  last: {last_lap:.2f} s" if last_lap
                   else (f"  |  Lap {lap_count}" if lap_count else ""))
        time_txt.set_text(f"t = {t:.2f} s{lap_str}")
        state_txt.set_text(fsm_state.name); state_txt.set_color(sc)

        fig.canvas.draw_idle(); fig.canvas.flush_events()

    return update, fig, cp_circles, cp_rings

# ── Shared episode runner (headless + tuner) ──────────────────────────────────

def run_episode(track_arr, fname, params: dict, max_time: float,
                stop_after_laps=0, verbose=False):
    pid_s = params.get("pid_straight",    C_.PID_STRAIGHT)
    pid_c = params.get("pid_corner",      C_.PID_CORNER)
    pid_sh= params.get("pid_sharp",       C_.PID_SHARP)
    pid_l = params.get("pid_lost",        C_.PID_LOST)

    gains = {State.STRAIGHT: pid_s, State.CORNER: pid_c,
             State.SHARP: pid_sh,   State.LOST:   pid_l}
    fsm   = FSM(gains=gains, fsm_params=params)

    sx, sy, stheta = C_.SPAWNS.get(fname, (C_.MAP_W_M/2, C_.MAP_H_M/2, 0.0))
    checkpoints    = C_.CHECKPOINTS.get(fname, [])
    n_cp           = len(checkpoints)

    robot  = Robot(sx, sy, stheta)
    last_ey= 0.0
    ys_arr = (np.arange(P.N_SENSORS) - (P.N_SENSORS - 1) / 2) * P.SPACING_M

    START_R, MIN_D = 0.12, 0.25
    departed = False; lap_start_t = 0.0
    lap_times = []; lap_count = 0
    next_cp = 0; cp_hit_mask = [False]*n_cp; lap_invalid = False
    cp_hit_total = 0; dist_total = 0.0
    prev_pos = np.array([sx, sy], dtype=float)
    t = 0.0

    while t < max_time:
        readings = read_sensors(robot, track_arr)
        weights  = readings ** 2
        total_w  = weights.sum()
        e_y      = last_ey = float(np.dot(weights, ys_arr)/total_w) if total_w > C_.FSM_LOST_WEIGHT else last_ey
        e_norm   = e_y / P.HALF_SPAN

        w_cmd, v_cmd, state = fsm.update(e_norm, total_w, C_.DT)
        if state != State.LOST:
            w_max = 2.0 * v_cmd / P.WHEEL_BASE if v_cmd > 0 else 0.0
            w_cmd = float(np.clip(w_cmd, -w_max, w_max))

        robot.update(v_cmd - w_cmd*P.WHEEL_BASE/2, v_cmd + w_cmd*P.WHEEL_BASE/2)
        rx, ry = robot.pos
        dist_total += float(np.hypot(rx - prev_pos[0], ry - prev_pos[1]))
        prev_pos[:] = rx, ry

        if departed and not lap_invalid:
            for ci, (cx, cy, cr) in enumerate(checkpoints):
                if np.hypot(rx-cx, ry-cy) < cr:
                    if ci == next_cp:
                        cp_hit_mask[next_cp] = True; cp_hit_total += 1; next_cp += 1
                        if verbose: _ok(f"CP{next_cp}/{n_cp} at {t:.2f} s")
                    elif not cp_hit_mask[ci]:
                        lap_invalid = True
                        if verbose: _err(f"CP{ci+1} out of order — lap void")
                    break

        dist_s = np.hypot(rx-sx, ry-sy)
        if not departed and dist_s > MIN_D:
            departed = True; lap_start_t = t
        if departed and dist_s < START_R:
            if not lap_invalid and next_cp == n_cp:
                lt = t - lap_start_t; lap_times.append(lt); lap_count += 1
                if verbose: _ok(f"Lap {lap_count}: {lt:.2f} s")
                if stop_after_laps and lap_count >= stop_after_laps:
                    t += C_.DT; break
            elif verbose:
                _err(f"Lap void ({'wrong-order CP' if lap_invalid else f'{next_cp}/{n_cp} CPs'})")
            next_cp = 0; cp_hit_mask = [False]*n_cp; lap_invalid = False; departed = False

        t += C_.DT

    return {"lap_times": lap_times, "lap_count": lap_count, "t_end": t,
            "cp_hit_total": cp_hit_total, "dist_total": dist_total}

# ── Headless simulation ───────────────────────────────────────────────────────

def simulate_headless(track_path: str, stop_after_laps=0):
    track_arr = load_track(track_path)
    fname     = os.path.basename(track_path)
    sx, sy, stheta = C_.SPAWNS.get(fname, (C_.MAP_W_M/2, C_.MAP_H_M/2, 0.0))
    checkpoints    = C_.CHECKPOINTS.get(fname, [])
    n_cp           = len(checkpoints)

    if fname not in C_.SPAWNS: _warn(f"No spawn for '{fname}' — using centre.")
    print(); _header("Headless Run")
    _kv("Track",       fname)
    _kv("Checkpoints", str(n_cp))
    _kv("Max time",    f"{C_.MAX_TIME} s" + (f"  •  stop after {stop_after_laps} laps" if stop_after_laps else ""))
    _rule(); print()

    robot  = Robot(sx, sy, stheta)
    fsm    = FSM()
    last_ey= 0.0
    ys_arr = (np.arange(P.N_SENSORS) - (P.N_SENSORS - 1) / 2) * P.SPACING_M

    START_R, MIN_D = 0.12, 0.25
    departed = False; lap_start_t = 0.0; last_lap = None; lap_count = 0
    next_cp = 0; cp_hit_mask = [False]*n_cp; lap_invalid = False
    t = 0.0; next_print = 0.0; wall_start = _time.perf_counter()

    try:
        while t < C_.MAX_TIME:
            readings = read_sensors(robot, track_arr)
            weights  = readings ** 2; total_w = weights.sum()
            e_y      = last_ey = float(np.dot(weights, ys_arr)/total_w) if total_w > C_.FSM_LOST_WEIGHT else last_ey
            e_norm   = e_y / P.HALF_SPAN
            w_cmd, v_cmd, state = fsm.update(e_norm, total_w, C_.DT)
            if state != State.LOST:
                w_max = 2.0 * v_cmd / P.WHEEL_BASE if v_cmd > 0 else 0.0
                w_cmd = float(np.clip(w_cmd, -w_max, w_max))
            robot.update(v_cmd - w_cmd*P.WHEEL_BASE/2, v_cmd + w_cmd*P.WHEEL_BASE/2)
            rx, ry = robot.pos

            if departed and not lap_invalid:
                for ci, (cx, cy, cr) in enumerate(checkpoints):
                    if np.hypot(rx-cx, ry-cy) < cr:
                        if ci == next_cp:
                            cp_hit_mask[next_cp] = True; next_cp += 1
                            print(C.ERASE, end=""); _ok(f"CP{next_cp}/{n_cp} at {t:.2f} s")
                        elif not cp_hit_mask[ci]:
                            lap_invalid = True
                            print(C.ERASE, end=""); _err(f"CP{ci+1} out of order — lap void")
                        break

            dist_s = np.hypot(rx-sx, ry-sy)
            if not departed and dist_s > MIN_D: departed = True; lap_start_t = t
            if departed and dist_s < START_R:
                print(C.ERASE, end="")
                if lap_invalid:
                    _err("Lap void (wrong-order checkpoint)")
                elif next_cp == n_cp:
                    last_lap = t - lap_start_t; lap_count += 1
                    _rule(); _ok(f"Lap {lap_count}  —  {last_lap:.2f} s"); _rule()
                    if stop_after_laps and lap_count >= stop_after_laps:
                        t += C_.DT; break
                else:
                    _warn(f"Finish crossed — only {next_cp}/{n_cp} CPs hit")
                next_cp = 0; cp_hit_mask = [False]*n_cp; lap_invalid = False; departed = False

            if t >= next_print:
                elapsed = _time.perf_counter() - wall_start
                spd     = t / elapsed if elapsed > 0 else 0
                pct     = t / C_.MAX_TIME; bw = 24; filled = int(bw*pct)
                bar     = "█"*filled + "░"*(bw-filled)
                ls      = f"{last_lap:.2f} s" if last_lap else "  —  "
                sc      = {State.STRAIGHT:C.GREEN, State.CORNER:C.YELLOW,
                           State.SHARP:C.RED, State.LOST:C.DIM}.get(state, C.WHITE)
                print(f"{C.ERASE}  {C.DIM}[{C.RESET}{C.CYAN}{bar}{C.RESET}{C.DIM}]{C.RESET}"
                      f"  {C.DIM}t{C.RESET} {t:5.1f}s  {C.DIM}lap{C.RESET} {lap_count}"
                      f"  {C.DIM}best{C.RESET} {ls}  {sc}{state.name:<8}{C.RESET}"
                      f"  {C.DIM}{spd:.0f}×{C.RESET}", end="", flush=True)
                next_print += 1.0
            t += C_.DT

    except KeyboardInterrupt:
        pass

    wall_total = _time.perf_counter() - wall_start
    print(); print()
    _rule(); _ok(f"Done  —  t={t:.2f} s  |  {lap_count} lap(s)  |  {t/wall_total:.0f}× realtime  ({wall_total:.1f} s wall)"); _rule()
    print()

# ── Tuner ─────────────────────────────────────────────────────────────────────

def _score(res, max_time):
    times = res["lap_times"]
    if not times:
        return 1e5 - (res["cp_hit_total"]*2.0 + res["dist_total"]*0.5) + res["t_end"]
    best = min(times); avg = sum(times)/len(times)
    return best + 0.3*(avg-best) + 5.0/len(times)

def _sample(rng):
    s = {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in C_.SEARCH_SPACE.items()}
    s["str_enter"] = min(s["str_enter"], s["str_exit"]  - 0.05)
    s["shp_exit"]  = min(s["shp_exit"],  s["shp_enter"] - 0.05)
    for pfx in ("str", "cor", "shp"):
        ratio = s[f"{pfx}_kd"] / max(s[f"{pfx}_kp"], 1e-9)
        if ratio < 0.15: s[f"{pfx}_kd"] = s[f"{pfx}_kp"] * 0.15
        elif ratio > 2.0: s[f"{pfx}_kd"] = s[f"{pfx}_kp"] * 2.0
    s["cor_spd"] = min(s["cor_spd"], s["str_spd"])
    s["shp_spd"] = min(s["shp_spd"], s["cor_spd"])
    return s

def _params_from_sample(s):
    return {
        "pid_straight":     (s["str_kp"], 0.0, s["str_kd"], s["str_limit"], s["str_spd"]),
        "pid_corner":       (s["cor_kp"], 0.0, s["cor_kd"], s["cor_limit"], s["cor_spd"]),
        "pid_sharp":        (s["shp_kp"], 0.0, s["shp_kd"], s["shp_limit"], s["shp_spd"]),
        "pid_lost":         C_.PID_LOST,
        "deriv_alpha":      s["deriv_alpha"],
        "fsm_str_exit":     s["str_exit"],
        "fsm_str_enter":    s["str_enter"],
        "fsm_sharp_enter":  s["shp_enter"],
        "fsm_sharp_exit":   s["shp_exit"],
        "fsm_lost_search_w":s["lost_search_w"],
        "accel_rate":       s["accel_rate"],
    }

def _print_params(p):
    s, c, sh = p["pid_straight"], p["pid_corner"], p["pid_sharp"]
    def _pid(t): return f"kp={t[0]:.3f}  kd={t[2]:.3f}  lim={t[3]:.2f}  spd={t[4]:.3f} m/s"
    _kv("PID STRAIGHT", _pid(s),  C.GREEN)
    _kv("PID CORNER",   _pid(c),  C.YELLOW)
    _kv("PID SHARP",    _pid(sh), C.RED)
    _kv("DERIV_ALPHA",  f"{p['deriv_alpha']:.3f}")
    _kv("FSM STR",      f"exit={p['fsm_str_exit']:.3f}  enter={p['fsm_str_enter']:.3f}")
    _kv("FSM SHP",      f"enter={p['fsm_sharp_enter']:.3f}  exit={p['fsm_sharp_exit']:.3f}")
    _kv("LOST SEARCH W",f"{p['fsm_lost_search_w']:.3f} rad/s")
    _kv("ACCEL RATE",   f"{p['accel_rate']:.2f} m/s²")
    print(f"\n  {C.DIM}# paste into config.py ─────────────────────────────────────{C.RESET}")
    print(f"  {C.CYAN}PID_STRAIGHT = ({s[0]:.4f}, 0.0, {s[2]:.4f}, {s[3]:.4f}, {s[4]:.4f}){C.RESET}")
    print(f"  {C.CYAN}PID_CORNER   = ({c[0]:.4f}, 0.0, {c[2]:.4f}, {c[3]:.4f}, {c[4]:.4f}){C.RESET}")
    print(f"  {C.CYAN}PID_SHARP    = ({sh[0]:.4f}, 0.0, {sh[2]:.4f}, {sh[3]:.4f}, {sh[4]:.4f}){C.RESET}")
    print(f"  {C.CYAN}DERIV_ALPHA      = {p['deriv_alpha']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_STR_EXIT     = {p['fsm_str_exit']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_STR_ENTER    = {p['fsm_str_enter']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_SHARP_ENTER  = {p['fsm_sharp_enter']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_SHARP_EXIT   = {p['fsm_sharp_exit']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_LOST_SEARCH_W= {p['fsm_lost_search_w']:.4f}{C.RESET}")
    print(f"  {C.CYAN}ACCEL_RATE       = {p['accel_rate']:.4f}{C.RESET}")

def tune(track_path, n_samples=500, tune_time=60.0, top_k=5, seed=0):
    track_arr = load_track(track_path)
    fname     = os.path.basename(track_path)
    rng       = np.random.default_rng(seed)

    print(); _header("Uniform Random Search Tuner")
    _kv("Track",      fname)
    _kv("Samples",    str(n_samples))
    _kv("Sim budget", f"{tune_time} s / candidate")
    _kv("Seed",       str(seed))
    _rule(); print()

    results = []; wall_0 = _time.perf_counter()

    for i in range(n_samples):
        params = _params_from_sample(_sample(rng))
        res    = run_episode(track_arr, fname, params, max_time=tune_time)
        sc     = _score(res, tune_time)
        results.append((sc, params, res))

        elapsed = _time.perf_counter() - wall_0
        pct     = (i+1)/n_samples; bw = 24; filled = int(bw*pct)
        bar     = "█"*filled + "░"*(bw-filled)
        best    = min(r[0] for r in results)
        eta     = (elapsed/(i+1)) * (n_samples-i-1)
        print(f"{C.ERASE}  {C.DIM}[{C.RESET}{C.CYAN}{bar}{C.RESET}{C.DIM}]{C.RESET}"
              f"  {i+1:>{len(str(n_samples))}}/{n_samples}"
              f"  {C.DIM}ETA{C.RESET} {eta:4.0f}s"
              f"  {C.DIM}best{C.RESET} {best:7.2f}"
              f"  {C.DIM}laps{C.RESET} {max(r[2]['lap_count'] for r in results)}"
              f"  {C.DIM}cps{C.RESET} {max(r[2]['cp_hit_total'] for r in results)}",
              end="", flush=True)

    print(); results.sort(key=lambda x: x[0])
    wall_total = _time.perf_counter() - wall_0
    print(); _rule()
    _ok(f"Search complete  —  {n_samples} samples  in  {wall_total:.1f} s  ({wall_total/n_samples*1000:.0f} ms/sample)")
    _rule(); print(); _header(f"Top {top_k} Results  —  paste into config.py")

    for rank, (sc, p, res) in enumerate(results[:top_k], 1):
        times = res["lap_times"]
        best_t = f"{min(times):.2f} s" if times else "—"
        avg_t  = f"{sum(times)/len(times):.2f} s" if times else "—"
        print(); _section(f"Rank {rank}  —  score {sc:.3f}  |  laps {res['lap_count']}  |  best {best_t}  |  avg {avg_t}")
        _print_params(p)

    best_sc, best_p, best_res = results[0]
    out_path = os.path.join(os.path.dirname(track_path), "tuner_best.txt")
    try:
        with open(out_path, "w") as f:
            s, c, sh = best_p["pid_straight"], best_p["pid_corner"], best_p["pid_sharp"]
            times = best_res["lap_times"]
            f.write(f"# Tuner: {n_samples} samples, {tune_time}s budget\n")
            f.write(f"# Score {best_sc:.4f}  |  Laps {best_res['lap_count']}"
                    + (f"  |  Best {min(times):.2f} s\n\n" if times else "\n\n"))
            f.write(f"PID_STRAIGHT = ({s[0]:.4f}, 0.0, {s[2]:.4f}, {s[3]:.4f}, {s[4]:.4f})\n")
            f.write(f"PID_CORNER   = ({c[0]:.4f}, 0.0, {c[2]:.4f}, {c[3]:.4f}, {c[4]:.4f})\n")
            f.write(f"PID_SHARP    = ({sh[0]:.4f}, 0.0, {sh[2]:.4f}, {sh[3]:.4f}, {sh[4]:.4f})\n")
            f.write(f"DERIV_ALPHA      = {best_p['deriv_alpha']:.4f}\n")
            f.write(f"FSM_STR_EXIT     = {best_p['fsm_str_exit']:.4f}\n")
            f.write(f"FSM_STR_ENTER    = {best_p['fsm_str_enter']:.4f}\n")
            f.write(f"FSM_SHARP_ENTER  = {best_p['fsm_sharp_enter']:.4f}\n")
            f.write(f"FSM_SHARP_EXIT   = {best_p['fsm_sharp_exit']:.4f}\n")
            f.write(f"FSM_LOST_SEARCH_W= {best_p['fsm_lost_search_w']:.4f}\n")
            f.write(f"ACCEL_RATE       = {best_p['accel_rate']:.4f}\n")
        print(); _ok(f"Best params saved → {out_path}")
    except Exception as e:
        _warn(f"Could not write output file: {e}")
    print()

# ── GUI simulation ────────────────────────────────────────────────────────────

def simulate(track_path: str):
    track_arr = load_track(track_path)
    fname     = os.path.basename(track_path)
    sx, sy, stheta = C_.SPAWNS.get(fname, (C_.MAP_W_M/2, C_.MAP_H_M/2, 0.0))
    checkpoints    = C_.CHECKPOINTS.get(fname, [])
    n_cp           = len(checkpoints)

    if fname not in C_.SPAWNS: _warn(f"No spawn for '{fname}' — using centre.")
    print(); _header("Line-Following Robot Simulator")
    _kv("Track",       fname)
    _kv("Spawn",       f"({sx:.2f}, {sy:.2f})  θ={stheta:.2f} rad")
    _kv("Checkpoints", str(n_cp))
    _kv("States",      "STRAIGHT → CORNER → SHARP | LOST")
    _rule(); print()

    robot  = Robot(sx, sy, stheta)
    fsm    = FSM()
    last_ey= 0.0
    ys_arr = (np.arange(P.N_SENSORS) - (P.N_SENSORS - 1) / 2) * P.SPACING_M

    START_R, MIN_D = 0.12, 0.25
    departed = False; lap_start_t = 0.0; last_lap = None; lap_count = 0
    next_cp = 0; cp_hit_mask = [False]*n_cp; lap_invalid = False

    update_viz, fig, cp_circles, cp_rings = build_dashboard(track_arr, (sx, sy), checkpoints)
    step = 0; t = 0.0

    try:
        while t < C_.MAX_TIME and plt.fignum_exists(fig.number):
            readings = read_sensors(robot, track_arr)
            weights  = readings**2; total_w = weights.sum()
            e_y      = last_ey = float(np.dot(weights, ys_arr)/total_w) if total_w > C_.FSM_LOST_WEIGHT else last_ey
            e_norm   = e_y / P.HALF_SPAN
            w_cmd, v_cmd, state = fsm.update(e_norm, total_w, C_.DT)
            if state != State.LOST:
                w_max = 2.0*v_cmd/P.WHEEL_BASE if v_cmd > 0 else 0.0
                w_cmd = float(np.clip(w_cmd, -w_max, w_max))
            robot.update(v_cmd - w_cmd*P.WHEEL_BASE/2, v_cmd + w_cmd*P.WHEEL_BASE/2)
            rx, ry = robot.pos

            if departed and not lap_invalid:
                for ci, (cx, cy, cr) in enumerate(checkpoints):
                    if np.hypot(rx-cx, ry-cy) < cr:
                        if ci == next_cp:
                            cp_hit_mask[next_cp] = True; _ok(f"CP{next_cp+1}/{n_cp} at {t:.2f} s"); next_cp += 1
                        elif not cp_hit_mask[ci]:
                            lap_invalid = True; _err(f"CP{ci+1} out of order — lap void")
                        break

            dist_s = np.hypot(rx-sx, ry-sy)
            if not departed and dist_s > MIN_D: departed = True; lap_start_t = t
            if departed and dist_s < START_R:
                if lap_invalid: _err("Lap void (wrong-order checkpoint)")
                elif next_cp == n_cp:
                    last_lap = t - lap_start_t; lap_count += 1
                    _rule(); _ok(f"Lap {lap_count}  —  {last_lap:.2f} s"); _rule()
                else: _warn(f"Finish crossed — only {next_cp}/{n_cp} CPs hit")
                next_cp = 0; cp_hit_mask = [False]*n_cp; lap_invalid = False; departed = False

            if step % 4 == 0:
                update_viz(robot, readings, e_y, t, lap_count, last_lap, state, cp_hit_mask, lap_invalid)
            t += C_.DT; step += 1

    except KeyboardInterrupt:
        pass

    print(); _rule(); _ok(f"Simulation ended  —  t={t:.2f} s  |  {lap_count} lap(s)"); _rule(); print()
    plt.ioff(); plt.show()

