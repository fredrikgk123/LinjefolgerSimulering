"""
Line-Following Robot Simulator
================================
Run:
    python main.py                          # uses bane_fase2.png
    python main.py --track assets/suzuka.png

Controls (while the window is open):
    Close the window to stop the simulation.
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("TkAgg")          # change to "Qt5Agg" if TkAgg is unavailable
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageFilter

# ──────────────────────────────────────────────────────────────────────────────
# PARAMETERS  (edit these to tune the robot)
# ──────────────────────────────────────────────────────────────────────────────

# Simulation
DT           = 0.005        # timestep  (s)  = 200 Hz  — matches ESP32 RC-sensor loop
MAX_TIME     = 40.0         # hard stop  (s)
PX_PER_M     = 500          # pixels per metre
MAP_W_M      = 4.0          # track image width  in metres
MAP_H_M      = 2.0          # track image height in metres

# PID gains — each FSM state uses its own gain set
# Error is normalised: ±1 = robot at edge of the 8-sensor array (HALF_SPAN = 14 mm)
#
# The error plot shows a ~3s slow square wave at ±12mm — the robot is making
# large full-span swings. This is an UNDERDAMPED system, not noise oscillation.
# Root cause: TAU_YAW = 390ms means the robot cannot respond faster than ~0.4s.
# Any KP that produces w_cmd > ~5 rad/s will cause the robot to overshoot the
# line before the turn registers, then correct back over — a slow square wave.
#
# Stability condition (Ziegler-Nichols approximation for a first-order lag):
#   KP_critical ≈ TAU_YAW / (DT × HALF_SPAN × PID_LIMIT / MAX_WHEEL)
#              ≈ 0.39 / (0.005 × ... ) — keep KP × DT / TAU_YAW < 0.15 (conservative)
#   KP_max ≈ 0.15 × 0.390 / 0.005 = 11.7  → use ≤ 6 to be safe
#
# KD must be high relative to KP to damp the slow oscillation.
# Rule: KD ≈ TAU_YAW × KP / 2  (critical damping for a first-order lag)
#   STRAIGHT: KD ≈ 0.39 × 3.0 / 2 = 0.59 → use 0.6
#   CORNER:   KD ≈ 0.39 × 5.0 / 2 = 0.98 → use 1.0
#   SHARP:    KD ≈ 0.39 × 6.0 / 2 = 1.17 → use 1.2
#
# KI = 0 everywhere — add only after oscillation is resolved.
#
#              KP     KI    KD    LIMIT   SPD (m/s)
PID_STRAIGHT = (3.0,  0.0,  0.6,  24.4,  1.20)   # very gentle — stay on line at speed
PID_CORNER   = (5.0,  0.0,  1.0,  24.4,  0.80)   # moderate corner correction
PID_SHARP    = (6.0,  0.0,  1.2,  24.4,  0.50)   # max authority, slow, heavily damped
PID_LOST     = (0.0,  0.0,  0.0,  24.4,  0.20)   # slow creep + rotate-and-search

INTEG_LIMIT  =  0.08        # kept for when KI is re-introduced later
DERIV_ALPHA  =  0.05        # very heavy low-pass on derivative — filters sensor noise
                             # before it feeds into the already-slow yaw response

# FSM transition thresholds
# The error plot shows the robot spending ALL its time at ±12mm (e_norm ≈ ±0.86).
# FSM_STR_EXIT = 0.25 is being crossed immediately on every correction,
# so the robot never settles in STRAIGHT. Raise it significantly.
#
# HALF_SPAN = 14 mm:
#   e_norm 0.15 → 2.1 mm offset  ← STRAIGHT re-entry (tight band around zero)
#   e_norm 0.40 → 5.6 mm offset  ← STRAIGHT exit (must exceed this to leave)
#   e_norm 0.80 → 11.2 mm offset ← SHARP entry (near full deflection)
#   e_norm 0.65 → 9.1 mm offset  ← SHARP exit
#
FSM_STR_EXIT    = 0.40   # STRAIGHT → CORNER  — must be off-centre by 5.6mm to leave
FSM_STR_ENTER   = 0.15   # CORNER → STRAIGHT  — must be within 2.1mm of centre

FSM_SHARP_ENTER = 0.80   # CORNER → SHARP     — near edge of array (11.2mm)
FSM_SHARP_EXIT  = 0.65   # SHARP  → CORNER

FSM_LOST_WEIGHT  = 0.002  # → LOST  (very low: sensor sweeps wide in corners due to 115mm fwd offset)
FSM_FOUND_WEIGHT = 0.015  # LOST → CORNER when weight rises above this

FSM_LOST_SEARCH_W = 3.0   # rad/s — slow deliberate search sweep

ACCEL_RATE   = 40.0         # m/s²  — gentle ramp to avoid weight-transfer grip loss

# Robot hardware
# Pololu Micro Metal 6V HP 30:1 at 7.4 V, 34 mm wheels (r = 17 mm)
WHEEL_BASE   = 0.180        # m    — measured centre-to-centre of drive wheels
WHEEL_RADIUS = 0.017        # m    — standard 34 mm Pololu wheels
MAX_WHEEL    = 2.20         # m/s  — no-load: 1233 RPM × 2π/60 × 0.017 m
MOTOR_TAU    = 0.055        # s    — mechanical time constant (HP series, loaded ≈ 50–70 ms)
DEADZONE     = 0.04         # m/s  — minimum effective wheel speed

# Sensor array — Pololu QTRX-HD-25RC  (central 8 of 25 channels wired)
#   8 sensors, 4 mm pitch, active span = 28 mm
#   RC mode: pulse width 300–2500 µs (mapped to 0–1 in simulation)
#   Sensor bar is 160 mm ahead of the drive axle.
#   CoM is 45 mm ahead of axle → SENSOR_FWD = 160 - 45 = 115 mm from CoM.
N_SENSORS    = 8
SPACING_M    = 0.004        # m  — 4 mm pitch (datasheet)
SENSOR_FWD   = 0.115        # m  — sensor bar offset from CoM (160 mm - 45 mm CoM offset)
NOISE_STD    = 0.003        # Gaussian σ on normalised readings (RC mode is low-noise)
HALF_SPAN    = ((N_SENSORS - 1) / 2) * SPACING_M   # 0.014 m — normalisation denominator

# ── Advanced physics ──────────────────────────────────────────────────────────
# Measured / derived from physical robot description.
#
# Geometry (inverted-T chassis):
#   Wheels sit wider than the body. Axle-to-front = 160 mm (sensor end).
#   Robot rests on screw heads at front → effectively a rear-axle drive with
#   passive front contact points at the sensor bar end.
#   Total chassis height < 8 mm (very flat), sensor lifted 5 mm off floor.
#
# Moment of inertia — inverted-T shape, modelled as two rectangles:
#   Crossbar (wheel section): m_c = 0.5×M, L=0.18 m wide, W=0.04 m deep
#   Spine    (body+sensor):   m_s = 0.5×M, L=0.16 m long, W=0.04 m wide
#   Iz = Iz_crossbar + Iz_spine  (parallel-axis theorem, CoM at 45 mm from axle)
ROBOT_MASS   = 0.270        # kg   — measured total mass

_M = ROBOT_MASS
# Crossbar: centred on axle, width=WHEEL_BASE, depth=40mm
_Iz_cross = (_M * 0.5) / 12.0 * (0.180**2 + 0.040**2)
# Spine: from axle to sensor end (160mm long, 40mm wide), CoM at 80mm from axle
# Parallel-axis shift: d = |80mm - 45mm| = 35mm from robot CoM
_Iz_spine = (_M * 0.5) / 12.0 * (0.160**2 + 0.040**2) + (_M * 0.5) * 0.035**2
ROBOT_IZ   = _Iz_cross + _Iz_spine
# ≈ 0.000854 kg·m²

# Centre-of-mass: 45 mm AHEAD of drive axle (measured).
# This is far forward — at high speed the drive wheels carry only a small
# fraction of the weight, severely limiting available traction.
COM_OFFSET   = 0.045        # m   — CoM forward of axle (measured)

# Weight on drive wheels at rest:
#   Taking moments about the front contact points (screw heads at sensor, 160 mm away):
#   F_axle × 0.160 = M×g × (0.160 - 0.045)  →  F_axle = M×g × 0.115/0.160
#   Fraction on drive wheels = 0.115 / 0.160 = 0.719
_AXLE_TO_FRONT  = 0.160     # m — axle to front contact (sensor end)
_COM_TO_FRONT   = _AXLE_TO_FRONT - COM_OFFSET          # 0.115 m
DRIVE_LOAD_REST = _COM_TO_FRONT / _AXLE_TO_FRONT       # ≈ 0.719

# Lateral tyre friction
# Measured tilt-plane method: μ ≈ 1.12 (tan of slip angle).
# This is the TOTAL friction; lateral component ≈ 0.85–0.90 × total
# because the wheel contact patch (22 mm wide) gives some anisotropy.
# Use 0.95 × 1.12 = 1.06 as the lateral coefficient.
MU_LATERAL      = 1.06      # —   lateral friction coefficient (measured × 0.95)
G               = 9.81      # m/s²

# Wheel contact patch: 22 mm wide. At speed the effective grip area matters
# for slip modelling but is already captured in MU_LATERAL above.
CONTACT_WIDTH   = 0.022     # m — for documentation / future slip model

# Motor back-EMF / torque model
# Pololu 6V HP 30:1: Ke = 6V / (1000 RPM × 2π/60) = 0.0573 V·s/rad
MOTOR_KE     = 0.0573       # V·s/rad — back-EMF constant
MOTOR_KT     = 0.0573       # N·m/A   — torque constant (= Ke in SI)
MOTOR_R      = 13.5         # Ω       — winding resistance (HP series)
MOTOR_V      = 7.4          # V       — supply voltage

# Spawn positions (x, y in metres from bottom-left, theta in radians)
SPAWNS = {
    "bane_fase2.png": (2.00, 0.14,  0.00),
    "suzuka.png":     (0.55, 0.68, -0.40),
}

# ──────────────────────────────────────────────────────────────────────────────
# TRACK LOADER
# ──────────────────────────────────────────────────────────────────────────────

def load_track(path: str) -> np.ndarray:
    """Load a grayscale track image and resize it to the simulator resolution."""
    img = Image.open(path).convert("L")
    arr = np.array(img)
    if arr.mean() < 128:          # dark background → invert
        arr = 255 - arr
        img = Image.fromarray(arr)
    img = img.resize(
        (int(MAP_W_M * PX_PER_M), int(MAP_H_M * PX_PER_M)),
        resample=getattr(Image, "Resampling", Image).LANCZOS)
    blurred = img.filter(ImageFilter.GaussianBlur(radius=2))
    return np.array(blurred, dtype=np.float32)

# ──────────────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────────────

def world_to_px(x, y):
    px = np.clip(x * PX_PER_M, 0, int(MAP_W_M * PX_PER_M) - 1)
    py = np.clip((MAP_H_M - y) * PX_PER_M, 0, int(MAP_H_M * PX_PER_M) - 1)
    return px, py


def bilinear(arr, x, y):
    x0, y0 = int(x), int(y)
    x1 = min(x0 + 1, arr.shape[1] - 1)
    y1 = min(y0 + 1, arr.shape[0] - 1)
    dx, dy = x - x0, y - y0
    return ((1-dx)*(1-dy)*arr[y0,x0] + dx*(1-dy)*arr[y0,x1]
            + (1-dx)*dy*arr[y1,x0] + dx*dy*arr[y1,x1])


from enum import Enum, auto

class State(Enum):
    STRAIGHT = auto()
    CORNER   = auto()
    SHARP    = auto()
    LOST     = auto()


class FSM:
    """
    Finite State Machine that selects PID gains and target speed.

    Transitions use hysteresis — every edge has a separate enter and exit
    threshold so the state cannot oscillate at a boundary.

    LOST state: instead of driving blind, the robot rotates slowly toward
    the last known error direction (the side the line was last seen on)
    while creeping forward at low speed.
    """

    STATE_COLOURS = {
        State.STRAIGHT: "#39d353",   # green
        State.CORNER:   "#ffa657",   # orange
        State.SHARP:    "#f78166",   # red
        State.LOST:     "#8b949e",   # grey
    }

    def __init__(self):
        self.state        = State.STRAIGHT
        self.integral     = 0.0
        self.prev_err     = 0.0
        self.d_filt       = 0.0
        self.v_filt       = 0.0
        self.last_e_sign  = 0.0   # sign of last valid error: +1 right, -1 left

    @staticmethod
    def _gains(state):
        return {
            State.STRAIGHT: PID_STRAIGHT,
            State.CORNER:   PID_CORNER,
            State.SHARP:    PID_SHARP,
            State.LOST:     PID_LOST,
        }[state]

    def _next_state(self, ae, total_weight):
        """Determine next state using hysteresis on all edges."""
        # Loss of line overrides everything
        if total_weight < FSM_LOST_WEIGHT:
            return State.LOST

        # Recovery from LOST — wait for a strong signal before resuming
        if self.state == State.LOST:
            return State.CORNER if total_weight >= FSM_FOUND_WEIGHT else State.LOST

        if self.state == State.STRAIGHT:
            if ae > FSM_SHARP_ENTER: return State.SHARP
            if ae > FSM_STR_EXIT:    return State.CORNER
            return State.STRAIGHT

        if self.state == State.CORNER:
            if ae > FSM_SHARP_ENTER:  return State.SHARP
            if ae < FSM_STR_ENTER:    return State.STRAIGHT
            return State.CORNER

        if self.state == State.SHARP:
            if ae < FSM_SHARP_EXIT: return State.CORNER
            return State.SHARP

        return self.state

    def update(self, e_norm, total_weight, dt):
        """
        Returns (w_cmd, v_cmd, state).
        In LOST state w_cmd is a slow rotation toward the last known line side.
        """
        ae        = abs(e_norm)
        new_state = self._next_state(ae, total_weight)

        if new_state != self.state:
            self.integral = 0.0   # reset integral on transition to prevent windup carry-over
            self.d_filt   = 0.0   # reset derivative filter to prevent kick on entry
            self.prev_err = e_norm  # seed prev_err to current value → d_raw=0 on first step
            self.state    = new_state

        # Remember which side the line was last on (for LOST search direction)
        if total_weight >= FSM_FOUND_WEIGHT and e_norm != 0.0:
            self.last_e_sign = 1.0 if e_norm > 0 else -1.0

        kp, ki, kd, limit, target_speed = self._gains(self.state)

        if self.state == State.LOST:
            # Rotate toward the last known line direction; no forward PID
            w_cmd = self.last_e_sign * FSM_LOST_SEARCH_W
            # Do NOT update prev_err — avoids a derivative spike on recovery
        else:
            p             = kp * e_norm
            self.integral = float(np.clip(self.integral + e_norm * dt,
                                          -INTEG_LIMIT, INTEG_LIMIT))
            i             = ki * self.integral
            d_raw         = (e_norm - self.prev_err) / dt
            self.d_filt   = DERIV_ALPHA * d_raw + (1 - DERIV_ALPHA) * self.d_filt
            d             = kd * self.d_filt
            w_cmd         = float(np.clip(p + i + d, -limit, limit))
            self.prev_err = e_norm

        # Speed ramp: drop instantly when slowing, ramp up gradually
        if self.v_filt > target_speed:
            self.v_filt = target_speed
        else:
            self.v_filt = min(target_speed, self.v_filt + ACCEL_RATE * dt)

        return w_cmd, self.v_filt, self.state

# ──────────────────────────────────────────────────────────────────────────────
# ROBOT PHYSICS
# ──────────────────────────────────────────────────────────────────────────────

class Robot:
    """
    Differential-drive robot with:
      - Back-EMF motor model  (torque drops with speed, not just first-order lag)
      - Yaw moment of inertia (angular velocity cannot change instantly)
      - CoM offset            (weight shifts off drive wheels under acceleration)
      - Lateral friction cap  (prevents physically impossible cornering speeds)
    """
    def __init__(self, x, y, theta):
        self.pos   = np.array([x, y], dtype=float)
        self.theta = float(theta)
        self.vL    = 0.0    # actual left  wheel speed  (m/s)
        self.vR    = 0.0    # actual right wheel speed  (m/s)
        self.omega = 0.0    # actual yaw rate            (rad/s)
        self.v     = 0.0    # actual forward speed       (m/s)

    # ── motor model ───────────────────────────────────────────────────────────
    @staticmethod
    def _motor_accel(v_wheel, v_cmd):
        """
        Compute wheel acceleration (m/s²) using a back-EMF torque model.

        The PWM duty cycle is proportional to v_cmd / MAX_WHEEL, giving
        an effective drive voltage of v_cmd / MAX_WHEEL × MOTOR_V.
        Net voltage = V_drive - back-EMF → current → torque → acceleration.
        """
        v_drive     = (v_cmd / MAX_WHEEL) * MOTOR_V   # effective drive voltage (V)
        omega_wheel = v_wheel / WHEEL_RADIUS           # current wheel speed (rad/s)
        back_emf    = MOTOR_KE * omega_wheel           # back-EMF (V)
        v_net       = np.clip(v_drive - back_emf, -MOTOR_V, MOTOR_V)
        current     = v_net / MOTOR_R                  # A
        torque      = MOTOR_KT * current               # N·m
        m_eff       = ROBOT_MASS / 2.0
        accel       = (torque / WHEEL_RADIUS) / m_eff  # m/s²
        return float(np.clip(accel, -800.0, 800.0))

    # ── main update ───────────────────────────────────────────────────────────
    def update(self, vL_cmd, vR_cmd):
        # 1. Dead zone
        if abs(vL_cmd) < DEADZONE: vL_cmd = 0.0
        if abs(vR_cmd) < DEADZONE: vR_cmd = 0.0

        # 2. Back-EMF motor model — integrate wheel speeds from torque
        aL = self._motor_accel(self.vL, vL_cmd)
        aR = self._motor_accel(self.vR, vR_cmd)
        self.vL = float(np.clip(self.vL + aL * DT, -MAX_WHEEL, MAX_WHEEL))
        self.vR = float(np.clip(self.vR + aR * DT, -MAX_WHEEL, MAX_WHEEL))

        # 3. Kinematics
        v_demand     = 0.5 * (self.vL + self.vR)
        omega_demand = (self.vR - self.vL) / WHEEL_BASE

        # 4. Longitudinal weight transfer onto/off the drive wheels
        #    Robot geometry:
        #      - Drive axle at origin
        #      - Front contact (screw heads) at +160 mm
        #      - CoM at +45 mm
        #      - h_com ≈ 4 mm (chassis < 8 mm tall, CoM near floor — very low CG)
        #
        #    At rest: fraction on drive wheels = COM_TO_FRONT / AXLE_TO_FRONT
        #                                      = 115/160 = 0.719
        #
        #    Under longitudinal acceleration a_x:
        #      ΔF_axle = -M × a_x × h_com / AXLE_TO_FRONT
        #      (acceleration transfers load FORWARD → away from drive wheels)
        #
        #    This is the critical physics: hard acceleration UNLOADS the drive
        #    wheels, reducing grip exactly when the robot needs it most.
        h_com = 0.004           # m — CoM height (chassis ~8mm, CoM at ~half)
        a_fwd = (v_demand - self.v) / DT
        delta_load = -(ROBOT_MASS * a_fwd * h_com) / (_AXLE_TO_FRONT * ROBOT_MASS * G)
        drive_load_fraction = float(np.clip(DRIVE_LOAD_REST + delta_load, 0.10, 1.0))
        # Normal force on drive wheels
        F_normal_drive = drive_load_fraction * ROBOT_MASS * G  # N

        # 5. Yaw dynamics — first-order lag with inertia-derived time constant
        #    TAU_YAW = Iz / (M × (B/2)²)  where B = WHEEL_BASE
        #    With inverted-T Iz ≈ 0.000854 kg·m² → TAU_YAW ≈ 0.391 s
        #    Bound between MOTOR_TAU (floor) and 0.40 s (cap)
        TAU_YAW = float(np.clip(
            ROBOT_IZ / (ROBOT_MASS * (WHEEL_BASE / 2.0) ** 2),
            MOTOR_TAU, 0.40))
        self.omega += (omega_demand - self.omega) * (DT / TAU_YAW)

        # 6. Lateral friction speed cap
        #    Max centripetal force = μ_lat × F_normal_drive
        #    Required centripetal force = M × v × |ω|
        #    Cap: v ≤ (μ_lat × F_normal_drive) / (M × |ω|)
        #
        #    Because the CoM is 45 mm AHEAD of the axle, the drive wheels
        #    carry only ~72% of the weight at rest — and less under acceleration.
        #    This makes the lateral grip cap tighter than a balanced robot.
        if abs(self.omega) > 1e-6:
            v_max_lateral = (MU_LATERAL * F_normal_drive) / (ROBOT_MASS * abs(self.omega))
            v_demand = float(np.clip(v_demand, -v_max_lateral, v_max_lateral))

        self.v = v_demand

        # 7. Integrate pose
        self.pos   += np.array([np.cos(self.theta), np.sin(self.theta)]) * self.v * DT
        self.theta += self.omega * DT

    def sensor_world_pos(self):
        """Return (N,2) world positions of the sensor array."""
        c, s = np.cos(self.theta), np.sin(self.theta)
        R    = np.array([[c, -s], [s, c]])
        ys   = (np.arange(N_SENSORS) - (N_SENSORS - 1) / 2) * SPACING_M
        body = np.stack([np.full(N_SENSORS, SENSOR_FWD), ys], axis=1)
        return (R @ body.T).T + self.pos

# ──────────────────────────────────────────────────────────────────────────────
# SENSOR READING
# ──────────────────────────────────────────────────────────────────────────────

_rng = np.random.default_rng(42)

def read_sensors(robot: Robot, track: np.ndarray) -> np.ndarray:
    readings = []
    for wx, wy in robot.sensor_world_pos():
        px, py = world_to_px(wx, wy)
        val = bilinear(track, px, py) / 255.0
        readings.append(1.0 - val)   # 1 = black line, 0 = white
    arr = np.array(readings)
    arr = np.clip(arr + _rng.standard_normal(N_SENSORS) * NOISE_STD, 0, 1)
    return np.round(arr * 1000) / 1000   # quantise to 10-bit

# ──────────────────────────────────────────────────────────────────────────────
# VISUALISER  (matplotlib live dashboard)
# ──────────────────────────────────────────────────────────────────────────────

def build_dashboard(track_arr: np.ndarray, spawn_xy):
    BG, PANEL = "#0d1117", "#161b22"
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 7), facecolor=BG)
    fig.canvas.manager.set_window_title("Line-Following Robot Simulator")

    gs = fig.add_gridspec(3, 2, width_ratios=[2.2, 1],
                          hspace=0.55, wspace=0.30,
                          left=0.05, right=0.97, top=0.93, bottom=0.07)

    ax_map    = fig.add_subplot(gs[:, 0])
    ax_sensor = fig.add_subplot(gs[0, 1])
    ax_err    = fig.add_subplot(gs[1, 1])
    ax_spd    = fig.add_subplot(gs[2, 1])

    # --- track map ---
    ax_map.imshow(track_arr, cmap="gray", vmin=0, vmax=255,
                  extent=(0, MAP_W_M, 0, MAP_H_M),
                  interpolation="bilinear", aspect="equal")
    ax_map.set_facecolor(BG)
    ax_map.set_title("Robot Live View", color="white", fontsize=13)
    ax_map.set_xlabel("x (m)", color="#8b949e"); ax_map.set_ylabel("y (m)", color="#8b949e")
    ax_map.tick_params(colors="#8b949e")

    # spawn circle
    sx, sy = spawn_xy
    ax_map.add_patch(plt.Circle((sx, sy), 0.10, color="#00ff88", alpha=0.18, zorder=2))
    ax_map.add_patch(plt.Circle((sx, sy), 0.10, color="#00ff88", alpha=0.55,
                                 fill=False, lw=1.5, zorder=2))

    path_line,    = ax_map.plot([], [], color="#58a6ff", lw=1.8, alpha=0.9, zorder=3)
    robot_circle   = plt.Circle((sx, sy), WHEEL_BASE / 2, color="#f78166", zorder=7)
    heading_line, = ax_map.plot([], [], color="white", lw=2, zorder=8)
    sensor_scat    = ax_map.scatter(np.zeros(N_SENSORS), np.zeros(N_SENSORS),
                                    c=["#21262d"] * N_SENSORS, s=18, zorder=6)
    ax_map.add_patch(robot_circle)

    # --- sensor bar ---
    ax_sensor.set_facecolor(PANEL)
    ax_sensor.set_title("Sensor readings (8 of 25 channels)", color="white", fontsize=9)
    ax_sensor.set_ylim(-0.05, 1.1); ax_sensor.set_xlim(-0.5, N_SENSORS - 0.5)
    ax_sensor.tick_params(colors="#8b949e", labelsize=7)
    ax_sensor.set_xticks([0, N_SENSORS // 2, N_SENSORS - 1])
    sensor_bars = ax_sensor.bar(range(N_SENSORS), [0]*N_SENSORS,
                                 color="#21262d", width=0.8)

    # --- error plot ---
    ax_err.set_facecolor(PANEL)
    ax_err.set_title("Lateral error (m)", color="white", fontsize=9)
    ax_err.set_ylim(-HALF_SPAN * 1.3, HALF_SPAN * 1.3)
    ax_err.axhline(0, color="#484f58", lw=0.8)
    ax_err.tick_params(colors="#8b949e", labelsize=7)
    err_line, = ax_err.plot([], [], color="#d2a8ff", lw=1.2)

    # --- speed plot ---
    ax_spd.set_facecolor(PANEL)
    ax_spd.set_title("Wheel speeds (m/s)", color="white", fontsize=9)
    ax_spd.set_ylim(-0.2, MAX_WHEEL + 0.2)
    ax_spd.axhline(0, color="#484f58", lw=0.8)
    ax_spd.tick_params(colors="#8b949e", labelsize=7)
    vl_line, = ax_spd.plot([], [], color="#58a6ff", lw=1.2, label="Left")
    vr_line, = ax_spd.plot([], [], color="#ffa657", lw=1.2, label="Right")
    ax_spd.legend(fontsize=7, loc="upper right",
                  facecolor=PANEL, labelcolor="white", edgecolor="#30363d")

    # --- time text ---
    time_txt = ax_map.text(0.02, 0.97, "t = 0.00 s", transform=ax_map.transAxes,
                           color="white", fontsize=11, va="top",
                           bbox=dict(boxstyle="round,pad=0.3", facecolor=PANEL, alpha=0.7))

    # --- FSM state label ---
    state_txt = ax_map.text(0.98, 0.97, "STRAIGHT", transform=ax_map.transAxes,
                            color=FSM.STATE_COLOURS[State.STRAIGHT], fontsize=11,
                            va="top", ha="right", fontweight="bold",
                            bbox=dict(boxstyle="round,pad=0.3", facecolor=PANEL, alpha=0.7))

    plt.ion()
    plt.show()

    # ---- update closure ----
    xs, ys_traj   = [], []
    t_hist, e_hist = [], []
    vl_hist, vr_hist = [], []
    WINDOW = int(5.0 / DT)   # 5-second rolling window for charts

    def update(robot: Robot, readings, e_y, t, lap_count=0, last_lap=None, fsm_state=State.STRAIGHT):
        # path
        xs.append(robot.pos[0]); ys_traj.append(robot.pos[1])
        path_line.set_data(xs, ys_traj)

        # robot dot (colour = FSM state) + heading
        rx, ry = robot.pos
        state_colour = FSM.STATE_COLOURS[fsm_state]
        robot_circle.center = (rx, ry)
        robot_circle.set_color(state_colour)
        hx = rx + 0.05 * np.cos(robot.theta)
        hy = ry + 0.05 * np.sin(robot.theta)
        heading_line.set_data([rx, hx], [ry, hy])

        # sensors
        spos = robot.sensor_world_pos()
        sensor_scat.set_offsets(spos)
        colours = [state_colour if r > 0.3 else "#21262d" for r in readings]
        sensor_scat.set_color(colours)
        for bar, r in zip(sensor_bars, readings):
            bar.set_height(r)
            bar.set_color(state_colour if r > 0.3 else "#21262d")

        # error history
        t_hist.append(t); e_hist.append(e_y)
        t_w = t_hist[-WINDOW:]; e_w = e_hist[-WINDOW:]
        err_line.set_data(t_w, e_w)
        ax_err.set_xlim(max(0, t - 5), t + 0.1)

        # speed history
        vl_hist.append(robot.vL); vr_hist.append(robot.vR)
        vl_w = vl_hist[-WINDOW:]; vr_w = vr_hist[-WINDOW:]
        vl_line.set_data(t_w, vl_w)
        vr_line.set_data(t_w, vr_w)
        ax_spd.set_xlim(max(0, t - 5), t + 0.1)

        # time + lap label
        lap_str = f"  |  Lap {lap_count}  —  last: {last_lap:.2f} s" if last_lap else (f"  |  Lap {lap_count}" if lap_count else "")
        time_txt.set_text(f"t = {t:.2f} s{lap_str}")

        # FSM state label
        state_txt.set_text(fsm_state.name)
        state_txt.set_color(state_colour)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    return update, fig

# ──────────────────────────────────────────────────────────────────────────────
# MAIN SIMULATION LOOP
# ──────────────────────────────────────────────────────────────────────────────

def simulate(track_path: str):
    track_arr = load_track(track_path)
    fname     = os.path.basename(track_path)

    if fname in SPAWNS:
        sx, sy, stheta = SPAWNS[fname]
    else:
        sx, sy, stheta = MAP_W_M / 2, MAP_H_M / 2, 0.0
        print(f"[warn] No spawn entry for '{fname}' — using centre.")

    print(f"Track : {track_path}")
    print(f"Spawn : ({sx:.2f}, {sy:.2f})  θ={stheta:.2f} rad")
    print(f"States: STRAIGHT (green) → CORNER (orange) → SHARP (red) | LOST (grey)")
    print(f"Press Ctrl-C or close the window to stop.\n")

    robot   = Robot(sx, sy, stheta)
    fsm     = FSM()
    last_ey = 0.0

    # Lap timing
    START_RADIUS = 0.12     # m — radius of start/finish zone
    MIN_DEPART   = 0.25     # m — must travel this far before lap counts
    departed     = False
    lap_count    = 0
    lap_start_t  = 0.0
    last_lap_time = None

    update_viz, fig = build_dashboard(track_arr, (sx, sy))

    RENDER_EVERY = 4
    step = 0
    t    = 0.0

    try:
        while t < MAX_TIME and plt.fignum_exists(fig.number):
            readings = read_sensors(robot, track_arr)
            weights  = readings ** 2
            ys       = (np.arange(N_SENSORS) - (N_SENSORS - 1) / 2) * SPACING_M
            total_w  = weights.sum()

            # Line position estimate — use same threshold as FSM LOST trigger
            if total_w > FSM_LOST_WEIGHT:
                e_y = last_ey = float(np.dot(weights, ys) / total_w)
            else:
                e_y = last_ey   # hold last known position when line lost

            e_norm = e_y / HALF_SPAN

            # FSM → angular velocity command + target speed
            w_cmd, v_cmd, state = fsm.update(e_norm, total_w, DT)

            # Clamp steering so neither wheel reverses — skip in LOST so the
            # search rotation isn't bottlenecked by the slow crawl speed.
            if state != State.LOST:
                w_max = 2.0 * v_cmd / WHEEL_BASE if v_cmd > 0 else 0.0
                w_cmd = float(np.clip(w_cmd, -w_max, w_max))

            vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
            vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

            robot.update(vL_cmd, vR_cmd)

            # Lap detection
            dist_to_start = np.hypot(robot.pos[0] - sx, robot.pos[1] - sy)
            if not departed and dist_to_start > MIN_DEPART:
                departed    = True
                lap_start_t = t
            if departed and dist_to_start < START_RADIUS:
                last_lap_time = t - lap_start_t
                lap_count    += 1
                print(f"  Lap {lap_count}: {last_lap_time:.2f} s")
                departed = False

            if step % RENDER_EVERY == 0:
                update_viz(robot, readings, e_y, t, lap_count, last_lap_time, state)

            t    += DT
            step += 1

    except KeyboardInterrupt:
        pass

    print(f"\nSimulation ended at t = {t:.2f} s  ({lap_count} lap(s) completed).")
    plt.ioff()
    plt.show()


# ──────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Line-Following Robot Simulator")
    parser.add_argument("--track", default="assets/bane_fase2.png",
                        help="Path to track image (default: assets/bane_fase2.png)")
    args = parser.parse_args()

    path = args.track
    if not os.path.isabs(path):
        path = os.path.join(os.path.dirname(__file__), path)

    if not os.path.exists(path):
        sys.exit(f"Error: track file not found: {path}")

    simulate(path)

