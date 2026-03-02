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
# CONSOLE OUTPUT  (all print calls go through these helpers)
# ──────────────────────────────────────────────────────────────────────────────

_IS_TTY = sys.stdout.isatty()

class C:
    """ANSI colour codes — silently disabled when stdout is not a tty."""
    _USE = _IS_TTY
    RESET  = "\033[0m"   if _USE else ""
    BOLD   = "\033[1m"   if _USE else ""
    DIM    = "\033[2m"   if _USE else ""
    GREEN  = "\033[92m"  if _USE else ""
    YELLOW = "\033[93m"  if _USE else ""
    RED    = "\033[91m"  if _USE else ""
    CYAN   = "\033[96m"  if _USE else ""
    WHITE  = "\033[97m"  if _USE else ""
    BLUE   = "\033[94m"  if _USE else ""
    ERASE  = "\033[2K\r" if _USE else "\r"   # erase current line

W = 68   # total console width

def _rule(char="─"):
    print(C.DIM + char * W + C.RESET)

def _header(title: str, char="═"):
    bar = char * W
    pad = (W - len(title) - 2) // 2
    print(C.BOLD + C.CYAN + bar)
    print(" " * pad + f" {title} ")
    print(bar + C.RESET)

def _section(title: str):
    print(C.DIM + "─" * W + C.RESET)
    print(C.BOLD + f"  {title}" + C.RESET)

def _kv(key: str, val: str, colour=None):
    col = colour or C.WHITE
    print(f"  {C.DIM}{key:<18}{C.RESET}{col}{val}{C.RESET}")

def _ok(msg: str):
    print(f"  {C.GREEN}✔{C.RESET}  {msg}")

def _warn(msg: str):
    print(f"  {C.YELLOW}⚠{C.RESET}  {msg}")

def _err(msg: str):
    print(f"  {C.RED}✗{C.RESET}  {msg}")

# ──────────────────────────────────────────────────────────────────────────────
# PARAMETERS  (edit these to tune the robot)
# ──────────────────────────────────────────────────────────────────────────────

# Simulation
DT           = 0.005        # timestep  (s)  = 200 Hz  — matches ESP32 RC-sensor loop
MAX_TIME     = 80.0         # hard stop  (s)
PX_PER_M     = 500          # pixels per metre
MAP_W_M      = 4.0          # track image width  in metres
MAP_H_M      = 2.0          # track image height in metres

# PID gains — each FSM state uses its own gain set
# Error is normalised: ±1 = robot at edge of the 8-sensor array (HALF_SPAN = 14 mm)
#
# Oscillation diagnosis:
#   The robot draws a line perpendicular to the track — classic bang-bang overshoot.
#   This means KP is far too high relative to the yaw lag (TAU_YAW ≈ 390 ms).
#   The robot commands a large turn, the yaw builds up slowly, overshoots the line,
#   then commands an equally large turn the other way → square-wave oscillation.
#
# Fix strategy:
#   1. Drastically cut KP so the peak ω_cmd is well inside the stable range.
#   2. Raise KD proportionally — derivative must actively brake the approach to zero.
#   3. Raise DERIV_ALPHA so the filter actually passes current error changes.
#      At 0.05 the derivative was 95% stale history — effectively disabled.
#   4. Widen the STRAIGHT band so brief wobbles don't escalate to CORNER gains.
#   5. Slow the LOST search sweep — fast spinning makes recovery unpredictable.
#
# Stability estimate (first-order lag, critical-damping rule):
#   ω_cmd = KP × e_norm;  yaw settles in ≈ TAU_YAW = 0.39 s
#   For no overshoot: KP × DT / TAU_YAW < 0.07  →  KP < 0.07×0.39/0.005 = 5.5
#   With KD damping:  KP ≤ 2.0 is safe; KD ≈ TAU_YAW × KP (full critical damping)
#
#              KP     KI    KD    LIMIT   SPD (m/s)
PID_STRAIGHT = (0.8,  0.0,  0.35,  5.0,  0.90)   # underdamping killer: low KP, KD ≈ 0.44×KP×TAU_YAW/DT
PID_CORNER   = (1.4,  0.0,  0.70,  5.0,  0.60)   # KD/KP = 0.5 — critically damped for TAU_YAW=0.39s
PID_SHARP    = (2.0,  0.0,  1.10,  5.0,  0.35)   # overdamped entry, slow speed to stay on line
PID_LOST     = (0.0,  0.0,  0.0,   5.0,  0.10)   # slow creep + rotate-and-search

INTEG_LIMIT  =  0.06        # kept for when KI is re-introduced later
DERIV_ALPHA  =  0.35        # balanced: enough filtering for noise, enough response to damp oscillation

# FSM transition thresholds
# HALF_SPAN = 14 mm:
#   e_norm 0.20 → 2.8 mm  ← STRAIGHT re-entry
#   e_norm 0.40 → 5.6 mm  ← STRAIGHT exit (must be meaningfully off-centre)
#   e_norm 0.80 → 11.2 mm ← SHARP entry  (near the edge of the array)
#   e_norm 0.65 → 9.1 mm  ← SHARP exit   (hysteresis)
#
FSM_STR_EXIT    = 0.40   # STRAIGHT → CORNER  — 5.6 mm off-centre
FSM_STR_ENTER   = 0.20   # CORNER → STRAIGHT  — 2.8 mm (achievable, not a razor edge)

FSM_SHARP_ENTER = 0.80   # CORNER → SHARP     — 11.2 mm (almost at array edge)
FSM_SHARP_EXIT  = 0.65   # SHARP  → CORNER    — hysteresis gap

FSM_LOST_WEIGHT  = 0.002  # → LOST
FSM_FOUND_WEIGHT = 0.015  # LOST → CORNER

FSM_LOST_SEARCH_W = 1.2   # rad/s — very slow search; fast spinning skips over the line

ACCEL_RATE   = 15.0         # m/s²  — gentle ramp; hard acceleration unloads drive wheels

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

# Checkpoints per track — ordered list of (cx, cy, radius) in metres.
# The robot must pass through ALL checkpoints in sequence before the
# finish line counts as a completed lap.  Adjust positions to sit on
# the track centreline; radius = 0.12 m is a generous ~one-robot-length.
CHECKPOINTS = {
    "bane_fase2.png": [
        (3.30, 0.14, 0.12),   # CP1 — far-right straight (after first long run)
        (3.35, 1.75, 0.12),   # CP2 — top-right corner apex
        (3.75, 1.20, 0.12),   # CP3 — top-right corner apex
        (2.00, 0.50, 0.12),   # CP4 — top-right corner apex
        (0.25, 1.20, 0.12),   # CP5 — top-right corner apex
        (0.65, 1.75, 0.12),   # CP6 — top-centre straight
        (0.70, 0.14, 0.12),   # CP7 — top-left corner apex
    ],
    "suzuka.png": [
        (2.00, 0.68, 0.12),   # CP1
        (3.20, 1.20, 0.12),   # CP2
        (2.00, 1.60, 0.12),   # CP3
        (0.80, 1.20, 0.12),   # CP4
    ],
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

def build_dashboard(track_arr: np.ndarray, spawn_xy, checkpoints=None):
    BG, PANEL = "#0d1117", "#161b22"
    CP_PENDING = "#f0e040"   # yellow — not yet hit
    CP_HIT     = "#39d353"   # green  — hit this lap
    CP_MISSED  = "#f78166"   # red    — skipped / wrong order

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

    # spawn circle  (start / finish line marker)
    sx, sy = spawn_xy
    ax_map.add_patch(plt.Circle((sx, sy), 0.10, color="#00ff88", alpha=0.18, zorder=2))
    ax_map.add_patch(plt.Circle((sx, sy), 0.10, color="#00ff88", alpha=0.55,
                                 fill=False, lw=1.5, zorder=2))
    ax_map.text(sx, sy + 0.13, "S/F", color="#00ff88", fontsize=7,
                ha="center", va="bottom", zorder=2)

    # --- checkpoint markers ---
    cp_circles  = []   # inner filled circle (changes colour when hit)
    cp_rings    = []   # outer ring (fixed yellow dashed)
    if checkpoints:
        for i, (cx, cy, cr) in enumerate(checkpoints):
            ring = plt.Circle((cx, cy), cr, color=CP_PENDING, fill=False,
                               lw=1.2, linestyle="--", alpha=0.70, zorder=4)
            fill = plt.Circle((cx, cy), cr * 0.45, color=CP_PENDING,
                               alpha=0.25, zorder=4)
            ax_map.add_patch(ring)
            ax_map.add_patch(fill)
            ax_map.text(cx, cy, f"CP{i+1}", color=CP_PENDING, fontsize=7,
                        ha="center", va="center", fontweight="bold", zorder=5)
            cp_rings.append(ring)
            cp_circles.append(fill)

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

    def update(robot: Robot, readings, e_y, t, lap_count=0, last_lap=None,
               fsm_state=State.STRAIGHT, cp_hit_mask=None, lap_invalid=False):
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

        # checkpoint colours:
        #   green  = hit correctly this lap
        #   red    = lap is invalidated (wrong-order hit occurred)
        #   yellow = pending
        if cp_hit_mask is not None:
            for i, hit in enumerate(cp_hit_mask):
                if lap_invalid:
                    colour = CP_MISSED   # whole lap is red when invalid
                elif hit:
                    colour = CP_HIT
                else:
                    colour = CP_PENDING
                cp_circles[i].set_facecolor(colour)
                cp_circles[i].set_alpha(0.45 if (hit or lap_invalid) else 0.25)
                cp_rings[i].set_edgecolor(colour)

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

    return update, fig, cp_circles, cp_rings

# ──────────────────────────────────────────────────────────────────────────────
# SHARED EPISODE RUNNER  (used by headless mode AND the tuner)
# ──────────────────────────────────────────────────────────────────────────────

def _run_episode(track_arr, fname, params: dict, max_time: float,
                 stop_after_laps: int = 0, verbose: bool = False):
    """
    Run one simulation episode with the given parameter dict.
    Returns a dict with keys: lap_times, lap_count, t_end.

    params keys (all optional — falls back to module globals if absent):
        pid_straight, pid_corner, pid_sharp, pid_lost   — (kp,ki,kd,limit,spd) tuples
        deriv_alpha
        fsm_str_exit, fsm_str_enter
        fsm_sharp_enter, fsm_sharp_exit
        fsm_lost_search_w
        accel_rate
    """
    # ── unpack params with fallback to current globals ──────────────────────
    pid_straight    = params.get("pid_straight",    PID_STRAIGHT)
    pid_corner      = params.get("pid_corner",      PID_CORNER)
    pid_sharp       = params.get("pid_sharp",       PID_SHARP)
    pid_lost        = params.get("pid_lost",        PID_LOST)
    deriv_alpha     = params.get("deriv_alpha",     DERIV_ALPHA)
    fsm_str_exit    = params.get("fsm_str_exit",    FSM_STR_EXIT)
    fsm_str_enter   = params.get("fsm_str_enter",   FSM_STR_ENTER)
    fsm_sharp_enter = params.get("fsm_sharp_enter", FSM_SHARP_ENTER)
    fsm_sharp_exit  = params.get("fsm_sharp_exit",  FSM_SHARP_EXIT)
    lost_search_w   = params.get("fsm_lost_search_w", FSM_LOST_SEARCH_W)
    accel_rate      = params.get("accel_rate",      ACCEL_RATE)

    if fname in SPAWNS:
        sx, sy, stheta = SPAWNS[fname]
    else:
        sx, sy, stheta = MAP_W_M / 2, MAP_H_M / 2, 0.0

    checkpoints = CHECKPOINTS.get(fname, [])
    n_cp        = len(checkpoints)

    # ── inline FSM that uses local params instead of globals ─────────────────
    class _LocalFSM:
        _GAINS = {
            State.STRAIGHT: pid_straight,
            State.CORNER:   pid_corner,
            State.SHARP:    pid_sharp,
            State.LOST:     pid_lost,
        }
        def __init__(self):
            self.state       = State.STRAIGHT
            self.integral    = 0.0
            self.prev_err    = 0.0
            self.d_filt      = 0.0
            self.v_filt      = 0.0
            self.last_e_sign = 0.0

        def _next_state(self, ae, tw):
            if tw < FSM_LOST_WEIGHT:  return State.LOST
            if self.state == State.LOST:
                return State.CORNER if tw >= FSM_FOUND_WEIGHT else State.LOST
            if self.state == State.STRAIGHT:
                if ae > fsm_sharp_enter: return State.SHARP
                if ae > fsm_str_exit:    return State.CORNER
                return State.STRAIGHT
            if self.state == State.CORNER:
                if ae > fsm_sharp_enter: return State.SHARP
                if ae < fsm_str_enter:   return State.STRAIGHT
                return State.CORNER
            if self.state == State.SHARP:
                return State.CORNER if ae < fsm_sharp_exit else State.SHARP
            return self.state

        def update(self, e_norm, total_weight, dt):
            ae        = abs(e_norm)
            new_state = self._next_state(ae, total_weight)
            if new_state != self.state:
                self.integral = 0.0
                self.d_filt   = 0.0
                self.prev_err = e_norm
                self.state    = new_state
            if total_weight >= FSM_FOUND_WEIGHT and e_norm != 0.0:
                self.last_e_sign = 1.0 if e_norm > 0 else -1.0
            kp, ki, kd, limit, target_speed = self._GAINS[self.state]
            if self.state == State.LOST:
                w_cmd = self.last_e_sign * lost_search_w
            else:
                p             = kp * e_norm
                self.integral = float(np.clip(self.integral + e_norm * dt,
                                              -INTEG_LIMIT, INTEG_LIMIT))
                i             = ki * self.integral
                d_raw         = (e_norm - self.prev_err) / dt
                self.d_filt   = deriv_alpha * d_raw + (1 - deriv_alpha) * self.d_filt
                d             = kd * self.d_filt
                w_cmd         = float(np.clip(p + i + d, -limit, limit))
                self.prev_err = e_norm
            if self.v_filt > target_speed:
                self.v_filt = target_speed
            else:
                self.v_filt = min(target_speed, self.v_filt + accel_rate * dt)
            return w_cmd, self.v_filt, self.state

    robot   = Robot(sx, sy, stheta)
    fsm     = _LocalFSM()
    last_ey = 0.0

    START_RADIUS  = 0.12
    MIN_DEPART    = 0.25
    departed      = False
    lap_count     = 0
    lap_start_t   = 0.0
    lap_times     = []

    next_cp     = 0
    cp_hit_mask = [False] * n_cp
    lap_invalid = False

    cp_hit_total = 0       # total CP hits across all valid lap attempts
    dist_total   = 0.0     # total distance travelled (proxy for staying on track)
    prev_pos     = np.array([sx, sy], dtype=float)

    ys_arr = (np.arange(N_SENSORS) - (N_SENSORS - 1) / 2) * SPACING_M
    t      = 0.0

    while t < max_time:
        readings = read_sensors(robot, track_arr)
        weights  = readings ** 2
        total_w  = weights.sum()

        if total_w > FSM_LOST_WEIGHT:
            e_y = last_ey = float(np.dot(weights, ys_arr) / total_w)
        else:
            e_y = last_ey

        e_norm          = e_y / HALF_SPAN
        w_cmd, v_cmd, state = fsm.update(e_norm, total_w, DT)

        if state != State.LOST:
            w_max = 2.0 * v_cmd / WHEEL_BASE if v_cmd > 0 else 0.0
            w_cmd = float(np.clip(w_cmd, -w_max, w_max))

        robot.update(v_cmd - w_cmd * WHEEL_BASE / 2,
                     v_cmd + w_cmd * WHEEL_BASE / 2)
        rx, ry = robot.pos

        # Accumulate distance
        dist_total += float(np.hypot(rx - prev_pos[0], ry - prev_pos[1]))
        prev_pos[0], prev_pos[1] = rx, ry

        # Checkpoint detection
        if departed and not lap_invalid:
            for cp_idx, (cx, cy, cr) in enumerate(checkpoints):
                if np.hypot(rx - cx, ry - cy) < cr:
                    if cp_idx == next_cp:
                        cp_hit_mask[next_cp] = True
                        cp_hit_total += 1
                        next_cp += 1
                        if verbose:
                            print(f"  ✓ CP{next_cp} hit at t = {t:.2f} s")
                    elif not cp_hit_mask[cp_idx]:
                        lap_invalid = True
                        if verbose:
                            print(f"  ✗ CP{cp_idx+1} out of order at t = {t:.2f} s")
                    break

        # Lap detection
        dist_to_start = np.hypot(rx - sx, ry - sy)
        if not departed and dist_to_start > MIN_DEPART:
            departed    = True
            lap_start_t = t
        if departed and dist_to_start < START_RADIUS:
            if not lap_invalid and next_cp == n_cp:
                lt = t - lap_start_t
                lap_times.append(lt)
                lap_count += 1
                if verbose:
                    print(f"  ✔ Lap {lap_count}: {lt:.2f} s")
                if stop_after_laps and lap_count >= stop_after_laps:
                    t += DT
                    break
            elif verbose:
                reason = "wrong-order CP" if lap_invalid else f"only {next_cp}/{n_cp} CPs"
                print(f"  ✗ Lap not counted ({reason})")
            next_cp     = 0
            cp_hit_mask = [False] * n_cp
            lap_invalid = False
            departed    = False

        t += DT

    return {"lap_times": lap_times, "lap_count": lap_count, "t_end": t,
            "cp_hit_total": cp_hit_total, "dist_total": dist_total}


# ──────────────────────────────────────────────────────────────────────────────
# HEADLESS SIMULATION  (console only — no GUI, runs as fast as possible)
# ──────────────────────────────────────────────────────────────────────────────

def simulate_headless(track_path: str, stop_after_laps: int = 0):
    """
    Identical physics to simulate() but with no matplotlib.
    Prints a progress line every second of simulated time.
    """
    import time as _time

    track_arr = load_track(track_path)
    fname     = os.path.basename(track_path)

    if fname not in SPAWNS:
        _warn(f"No spawn entry for '{fname}' — using centre.")

    checkpoints = CHECKPOINTS.get(fname, [])
    n_cp        = len(checkpoints)
    sx, sy, _   = SPAWNS.get(fname, (MAP_W_M/2, MAP_H_M/2, 0.0))

    print()
    _header("Headless Run")
    _kv("Track",       os.path.basename(track_path))
    _kv("Checkpoints", str(n_cp))
    _kv("Max time",    f"{MAX_TIME} s" + (f"  •  stop after {stop_after_laps} lap(s)" if stop_after_laps else ""))
    _rule()
    print()

    robot   = Robot(sx, sy, SPAWNS.get(fname, (0,0,0.0))[2])
    fsm     = FSM()
    last_ey = 0.0

    START_RADIUS = 0.12
    MIN_DEPART   = 0.25
    departed     = False
    lap_count    = 0
    lap_start_t  = 0.0
    last_lap_time = None

    next_cp     = 0
    cp_hit_mask = [False] * n_cp
    lap_invalid = False

    ys_arr     = (np.arange(N_SENSORS) - (N_SENSORS - 1) / 2) * SPACING_M
    t          = 0.0
    next_print = 0.0
    wall_start = _time.perf_counter()

    try:
        while t < MAX_TIME:
            readings = read_sensors(robot, track_arr)
            weights  = readings ** 2
            total_w  = weights.sum()

            if total_w > FSM_LOST_WEIGHT:
                e_y = last_ey = float(np.dot(weights, ys_arr) / total_w)
            else:
                e_y = last_ey

            e_norm = e_y / HALF_SPAN
            w_cmd, v_cmd, state = fsm.update(e_norm, total_w, DT)

            if state != State.LOST:
                w_max = 2.0 * v_cmd / WHEEL_BASE if v_cmd > 0 else 0.0
                w_cmd = float(np.clip(w_cmd, -w_max, w_max))

            robot.update(v_cmd - w_cmd * WHEEL_BASE / 2,
                         v_cmd + w_cmd * WHEEL_BASE / 2)
            rx, ry = robot.pos

            if departed and not lap_invalid:
                for cp_idx, (cx, cy, cr) in enumerate(checkpoints):
                    if np.hypot(rx - cx, ry - cy) < cr:
                        if cp_idx == next_cp:
                            cp_hit_mask[next_cp] = True
                            next_cp += 1
                            print(C.ERASE, end="")   # clear status line before event
                            _ok(f"CP{next_cp}/{n_cp} at {t:.2f} s")
                        elif not cp_hit_mask[cp_idx]:
                            lap_invalid = True
                            print(C.ERASE, end="")
                            _err(f"CP{cp_idx+1} out of order  (expected CP{next_cp+1})  — lap void")
                        break

            dist_to_start = np.hypot(rx - sx, ry - sy)
            if not departed and dist_to_start > MIN_DEPART:
                departed    = True
                lap_start_t = t
            if departed and dist_to_start < START_RADIUS:
                print(C.ERASE, end="")
                if lap_invalid:
                    _err("Lap void  (wrong-order checkpoint)")
                elif next_cp == n_cp:
                    last_lap_time = t - lap_start_t
                    lap_count    += 1
                    _rule()
                    _ok(f"Lap {lap_count}  —  {last_lap_time:.2f} s")
                    _rule()
                    if stop_after_laps and lap_count >= stop_after_laps:
                        t += DT
                        break
                else:
                    _warn(f"Finish crossed  —  only {next_cp}/{n_cp} CPs hit")
                next_cp     = 0
                cp_hit_mask = [False] * n_cp
                lap_invalid = False
                departed    = False

            if t >= next_print:
                wall_elapsed = _time.perf_counter() - wall_start
                speed_factor = t / wall_elapsed if wall_elapsed > 0 else 0
                pct          = t / MAX_TIME
                bar_w        = 24
                filled       = int(bar_w * pct)
                bar          = "█" * filled + "░" * (bar_w - filled)
                lap_str      = f"{last_lap_time:.2f} s" if last_lap_time else "  —   "
                state_col    = {
                    State.STRAIGHT: C.GREEN,
                    State.CORNER:   C.YELLOW,
                    State.SHARP:    C.RED,
                    State.LOST:     C.DIM,
                }.get(state, C.WHITE)
                line = (
                    f"  {C.DIM}[{C.RESET}{C.CYAN}{bar}{C.RESET}{C.DIM}]{C.RESET}"
                    f"  {C.DIM}t{C.RESET} {t:5.1f}s"
                    f"  {C.DIM}lap{C.RESET} {lap_count}"
                    f"  {C.DIM}best{C.RESET} {lap_str}"
                    f"  {state_col}{state.name:<8}{C.RESET}"
                    f"  {C.DIM}{speed_factor:.0f}×{C.RESET}"
                )
                print(f"{C.ERASE}{line}", end="", flush=True)
                next_print += 1.0

            t += DT

    except KeyboardInterrupt:
        pass

    wall_total = _time.perf_counter() - wall_start
    print()   # end in-place line
    print()
    _rule()
    _ok(f"Done  —  t={t:.2f} s  |  {lap_count} lap(s)  |  {t/wall_total:.0f}× realtime  ({wall_total:.1f} s wall)")
    _rule()
    print()


# ──────────────────────────────────────────────────────────────────────────────
# UNIFORM RANDOM SEARCH TUNER
# ──────────────────────────────────────────────────────────────────────────────

# Search space — (low, high) for each tunable scalar.
# Physical constants and sensor geometry are NOT included.
# Constraints enforced after sampling:
#   fsm_str_enter  < fsm_str_exit
#   fsm_sharp_exit < fsm_sharp_enter
#   kd / kp ratio kept in [0.15, 2.0]  per state (stability filter)
#
# ── HOW TO NARROW THE SEARCH ─────────────────────────────────────────────────
# After a broad run you will have a best candidate (see tuner_best.txt).
# Narrow each range to ±20–30 % around that value to do a fine search.
#
# Example — broad run found:  str_kp=2.39  →  narrow to  (1.8, 3.0)
#                              str_kd=1.66  →  narrow to  (1.2, 2.1)
#
# Rules of thumb:
#   • Keep the range at least 2× the precision you care about (e.g. ±0.1).
#   • Never set low == high; the sampler needs room to explore.
#   • FSM thresholds matter less than KP/KD — widen them to let the search
#     spend budget on the PID gains instead.
#   • If the robot never completes a lap, widen everything and lower speeds.
# ─────────────────────────────────────────────────────────────────────────────
_SEARCH_SPACE = {
    # ── STRAIGHT state PID ───────────────────────────────────────────────────
    # kp : proportional gain  — too high → oscillation; too low → sluggish
    # kd : derivative gain    — damps the approach to zero; scale with kp
    # limit : max |ω_cmd| (rad/s) — hard cap on angular rate command
    # spd : target forward speed (m/s) — bounded by grip and motor limits
    "str_kp":    (0.3,  3.0),   # current best ≈ 0.8  →  narrow e.g. (0.5, 1.5)
    "str_kd":    (0.1,  2.0),   # current best ≈ 0.35 →  narrow e.g. (0.2, 0.8)
    "str_limit": (2.0,  8.0),   # rarely needs tuning; 4–6 is usually fine
    "str_spd":   (0.5,  1.8),   # current best ≈ 0.90 →  narrow e.g. (0.7, 1.2)

    # ── CORNER state PID ─────────────────────────────────────────────────────
    # Higher kp/kd than STRAIGHT; speed must be ≤ str_spd (enforced in sampler)
    "cor_kp":    (0.5,  5.0),   # current best ≈ 1.4  →  narrow e.g. (0.8, 2.5)
    "cor_kd":    (0.2,  3.0),   # current best ≈ 0.70 →  narrow e.g. (0.4, 1.4)
    "cor_limit": (2.0,  8.0),   # keep same as str_limit or slightly higher
    "cor_spd":   (0.3,  1.2),   # current best ≈ 0.60 →  narrow e.g. (0.4, 0.9)

    # ── SHARP state PID ──────────────────────────────────────────────────────
    # Highest authority; speed must be ≤ cor_spd (enforced in sampler)
    "shp_kp":    (1.0,  6.0),   # current best ≈ 2.0  →  narrow e.g. (1.2, 3.5)
    "shp_kd":    (0.5,  4.0),   # current best ≈ 1.10 →  narrow e.g. (0.7, 2.0)
    "shp_limit": (2.0,  8.0),   # usually fine at 5–6
    "shp_spd":   (0.15, 0.7),   # current best ≈ 0.35 →  narrow e.g. (0.2, 0.55)

    # ── FSM state-change thresholds (normalised error, 0–1) ──────────────────
    # str_exit  : how far off-centre before leaving STRAIGHT → CORNER
    # str_enter : how centred the robot must be to return to STRAIGHT
    # shp_enter : how far off-centre before escalating to SHARP
    # shp_exit  : hysteresis — must drop below this to leave SHARP
    # Constraint: str_enter < str_exit,  shp_exit < shp_enter  (auto-enforced)
    "str_exit":    (0.20, 0.60),  # current best ≈ 0.40 →  narrow e.g. (0.28, 0.55)
    "str_enter":   (0.05, 0.35),  # current best ≈ 0.20 →  narrow e.g. (0.10, 0.30)
    "shp_enter":   (0.60, 0.95),  # current best ≈ 0.80 →  narrow e.g. (0.65, 0.92)
    "shp_exit":    (0.45, 0.85),  # current best ≈ 0.65 →  narrow e.g. (0.50, 0.78)

    # ── Miscellaneous ─────────────────────────────────────────────────────────
    # deriv_alpha  : EMA weight on the derivative (0 = frozen, 1 = raw/noisy)
    #                too low → KD is dead; too high → noise causes twitching
    # lost_search_w: yaw rate while scanning for the line (rad/s)
    #                too high → sensor sweeps past the line; too low → slow recovery
    # accel_rate   : speed ramp (m/s²); lower protects rear-wheel grip on hard accel
    "deriv_alpha":    (0.10, 0.60),  # current best ≈ 0.35 →  narrow e.g. (0.20, 0.50)
    "lost_search_w":  (0.5,  2.5),   # current best ≈ 1.2  →  narrow e.g. (0.7, 1.8)
    "accel_rate":     (5.0,  40.0),  # current best ≈ 15.0 →  narrow e.g. (8.0, 25.0)
}


def _score(result: dict, max_time: float) -> float:
    """
    Lower is better.

    When no laps complete (common early in the search) the score still
    differentiates candidates using partial progress signals so the
    search is not completely flat:
      - more valid CPs hit across all attempts = better
      - more total departed distance covered = better (proxy for staying on track)

    When laps complete:
      - best lap time is primary
      - consistency (avg - best) is secondary
      - more laps = small bonus (more data = more trustworthy)
    """
    lap_times  = result["lap_times"]
    cp_hit     = result.get("cp_hit_total", 0)
    dist       = result.get("dist_total",   0.0)

    if not lap_times:
        # No laps — use partial progress to rank candidates
        # 1e5 base penalty; subtract partial credit so better candidates score lower
        partial = cp_hit * 2.0 + dist * 0.5
        return 1e5 - partial + result["t_end"]

    best        = min(lap_times)
    avg         = sum(lap_times) / len(lap_times)
    consistency = avg - best
    lap_bonus   = 5.0 / len(lap_times)   # bonus shrinks as more laps accumulate

    return best + 0.3 * consistency + lap_bonus


def _sample(rng: np.random.Generator) -> dict:
    """Draw one random sample from _SEARCH_SPACE, enforce constraints."""
    s = {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in _SEARCH_SPACE.items()}

    # FSM threshold constraints
    s["str_enter"]  = min(s["str_enter"],  s["str_exit"]  - 0.05)
    s["shp_exit"]   = min(s["shp_exit"],   s["shp_enter"] - 0.05)

    # KD/KP ratio constraints (prevent absurdly overdamped or underdamped combos)
    for prefix in ("str", "cor", "shp"):
        kp = s[f"{prefix}_kp"]
        kd = s[f"{prefix}_kd"]
        ratio = kd / max(kp, 1e-9)
        if ratio < 0.15:
            s[f"{prefix}_kd"] = kp * 0.15
        elif ratio > 2.0:
            s[f"{prefix}_kd"] = kp * 2.0

    # Corner/sharp must not be faster than straight
    s["cor_spd"] = min(s["cor_spd"], s["str_spd"])
    s["shp_spd"] = min(s["shp_spd"], s["cor_spd"])

    return s


def _params_from_sample(s: dict) -> dict:
    return {
        "pid_straight":     (s["str_kp"], 0.0, s["str_kd"], s["str_limit"], s["str_spd"]),
        "pid_corner":       (s["cor_kp"], 0.0, s["cor_kd"], s["cor_limit"], s["cor_spd"]),
        "pid_sharp":        (s["shp_kp"], 0.0, s["shp_kd"], s["shp_limit"], s["shp_spd"]),
        "pid_lost":         PID_LOST,
        "deriv_alpha":      s["deriv_alpha"],
        "fsm_str_exit":     s["str_exit"],
        "fsm_str_enter":    s["str_enter"],
        "fsm_sharp_enter":  s["shp_enter"],
        "fsm_sharp_exit":   s["shp_exit"],
        "fsm_lost_search_w": s["lost_search_w"],
        "accel_rate":       s["accel_rate"],
    }


def _print_params(p: dict, score: float, rank: int = 1):
    s, c, sh = p["pid_straight"], p["pid_corner"], p["pid_sharp"]
    def _pid(t): return f"kp={t[0]:.3f}  kd={t[2]:.3f}  lim={t[3]:.2f}  spd={t[4]:.3f} m/s"
    _kv("PID STRAIGHT", _pid(s),  C.GREEN)
    _kv("PID CORNER",   _pid(c),  C.YELLOW)
    _kv("PID SHARP",    _pid(sh), C.RED)
    _kv("DERIV_ALPHA",  f"{p['deriv_alpha']:.3f}")
    _kv("FSM STR EXIT", f"{p['fsm_str_exit']:.3f}   enter={p['fsm_str_enter']:.3f}")
    _kv("FSM SHP ENTER",f"{p['fsm_sharp_enter']:.3f}  exit={p['fsm_sharp_exit']:.3f}")
    _kv("LOST SEARCH W",f"{p['fsm_lost_search_w']:.3f} rad/s")
    _kv("ACCEL RATE",   f"{p['accel_rate']:.2f} m/s²")
    # paste-ready block
    print()
    print(f"  {C.DIM}# ── paste-ready ──────────────────────────────────────────{C.RESET}")
    print(f"  {C.CYAN}PID_STRAIGHT = ({s[0]:.4f}, 0.0, {s[2]:.4f}, {s[3]:.4f}, {s[4]:.4f}){C.RESET}")
    print(f"  {C.CYAN}PID_CORNER   = ({c[0]:.4f}, 0.0, {c[2]:.4f}, {c[3]:.4f}, {c[4]:.4f}){C.RESET}")
    print(f"  {C.CYAN}PID_SHARP    = ({sh[0]:.4f}, 0.0, {sh[2]:.4f}, {sh[3]:.4f}, {sh[4]:.4f}){C.RESET}")
    print(f"  {C.CYAN}DERIV_ALPHA  = {p['deriv_alpha']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_STR_EXIT    = {p['fsm_str_exit']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_STR_ENTER   = {p['fsm_str_enter']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_SHARP_ENTER = {p['fsm_sharp_enter']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_SHARP_EXIT  = {p['fsm_sharp_exit']:.4f}{C.RESET}")
    print(f"  {C.CYAN}FSM_LOST_SEARCH_W = {p['fsm_lost_search_w']:.4f}{C.RESET}")
    print(f"  {C.CYAN}ACCEL_RATE   = {p['accel_rate']:.4f}{C.RESET}")


def tune(track_path: str, n_samples: int = 500, tune_time: float = 30.0,
         top_k: int = 5, seed: int = 0):
    """
    Uniform Random Search over the PID / FSM parameter space.

    Each candidate is evaluated on `tune_time` seconds of simulation.
    The top-k results are printed at the end with ready-to-paste values.

    Args:
        track_path  : track image to tune on
        n_samples   : number of random candidates to evaluate
        tune_time   : simulated time budget per candidate (s)
        top_k       : how many best results to display
        seed        : RNG seed for reproducibility
    """
    import time as _time

    track_arr = load_track(track_path)
    fname     = os.path.basename(track_path)

    rng = np.random.default_rng(seed)

    print()
    _header("Uniform Random Search Tuner")
    _kv("Track",       os.path.basename(track_path))
    _kv("Samples",     str(n_samples))
    _kv("Sim budget",  f"{tune_time} s / candidate")
    _kv("Seed",        str(seed))
    _rule()
    print()

    results   = []
    wall_0    = _time.perf_counter()

    for i in range(n_samples):
        sample = _sample(rng)
        params = _params_from_sample(sample)
        res    = _run_episode(track_arr, fname, params,
                              max_time=tune_time, stop_after_laps=0, verbose=False)
        sc     = _score(res, tune_time)
        results.append((sc, params, res))

        # In-place progress bar every sample
        elapsed    = _time.perf_counter() - wall_0
        pct        = (i + 1) / n_samples
        bar_w      = 24
        filled     = int(bar_w * pct)
        bar        = "█" * filled + "░" * (bar_w - filled)
        best_score = min(r[0] for r in results)
        max_laps   = max(r[2]["lap_count"] for r in results)
        max_cps    = max(r[2]["cp_hit_total"] for r in results)
        eta        = (elapsed / (i + 1)) * (n_samples - i - 1)
        line = (
            f"  {C.DIM}[{C.RESET}{C.CYAN}{bar}{C.RESET}{C.DIM}]{C.RESET}"
            f"  {i+1:>{len(str(n_samples))}}/{n_samples}"
            f"  {C.DIM}ETA{C.RESET} {eta:4.0f}s"
            f"  {C.DIM}best{C.RESET} {best_score:7.2f}"
            f"  {C.DIM}laps{C.RESET} {max_laps}"
            f"  {C.DIM}cps{C.RESET} {max_cps}"
        )
        print(f"{C.ERASE}{line}", end="", flush=True)

    print()   # end progress line

    # Sort ascending (lower score = better)
    results.sort(key=lambda x: x[0])

    wall_total = _time.perf_counter() - wall_0
    print()
    _rule()
    _ok(f"Search complete  —  {n_samples} samples  in  {wall_total:.1f} s  ({wall_total/n_samples*1000:.0f} ms/sample)")
    _rule()
    print()
    _header(f"Top {top_k} Results  —  copy-paste into main.py")

    for rank, (sc, p, res) in enumerate(results[:top_k], start=1):
        laps   = res["lap_count"]
        times  = res["lap_times"]
        best_t = f"{min(times):.2f} s" if times else "  —  "
        avg_t  = f"{sum(times)/len(times):.2f} s" if times else "  —  "
        print()
        _section(f"Rank {rank}  —  score {sc:.3f}  |  laps {laps}  |  best {best_t}  |  avg {avg_t}")
        _print_params(p, sc, rank)

    # Write best result to a file for easy reference
    best_sc, best_p, best_res = results[0]
    out_path = os.path.join(os.path.dirname(track_path), "tuner_best.txt")
    try:
        with open(out_path, "w") as f:
            f.write(f"# Tuner result — {n_samples} samples, {tune_time}s budget\n")
            f.write(f"# Score: {best_sc:.4f}  |  Laps: {best_res['lap_count']}  "
                    f"|  Best lap: {min(best_res['lap_times']):.2f} s\n\n" if best_res['lap_times']
                    else f"# Score: {best_sc:.4f}  |  No valid laps\n\n")
            s, c, sh = best_p["pid_straight"], best_p["pid_corner"], best_p["pid_sharp"]
            f.write(f"PID_STRAIGHT = ({s[0]:.4f}, 0.0, {s[2]:.4f}, {s[3]:.4f}, {s[4]:.4f})\n")
            f.write(f"PID_CORNER   = ({c[0]:.4f}, 0.0, {c[2]:.4f}, {c[3]:.4f}, {c[4]:.4f})\n")
            f.write(f"PID_SHARP    = ({sh[0]:.4f}, 0.0, {sh[2]:.4f}, {sh[3]:.4f}, {sh[4]:.4f})\n")
            f.write(f"DERIV_ALPHA  = {best_p['deriv_alpha']:.4f}\n")
            f.write(f"FSM_STR_EXIT    = {best_p['fsm_str_exit']:.4f}\n")
            f.write(f"FSM_STR_ENTER   = {best_p['fsm_str_enter']:.4f}\n")
            f.write(f"FSM_SHARP_ENTER = {best_p['fsm_sharp_enter']:.4f}\n")
            f.write(f"FSM_SHARP_EXIT  = {best_p['fsm_sharp_exit']:.4f}\n")
            f.write(f"FSM_LOST_SEARCH_W = {best_p['fsm_lost_search_w']:.4f}\n")
            f.write(f"ACCEL_RATE   = {best_p['accel_rate']:.4f}\n")
        print()
        _ok(f"Best params saved → {out_path}")
    except Exception as e:
        _warn(f"Could not write output file: {e}")
    print()


# ──────────────────────────────────────────────────────────────────────────────
# MAIN SIMULATION LOOP  (with GUI)
# ──────────────────────────────────────────────────────────────────────────────

def simulate(track_path: str):
    track_arr = load_track(track_path)
    fname     = os.path.basename(track_path)

    if fname in SPAWNS:
        sx, sy, stheta = SPAWNS[fname]
    else:
        sx, sy, stheta = MAP_W_M / 2, MAP_H_M / 2, 0.0
        _warn(f"No spawn entry for '{fname}' — using centre.")

    checkpoints = CHECKPOINTS.get(fname, [])
    n_cp        = len(checkpoints)

    print()
    _header("Line-Following Robot Simulator")
    _kv("Track",       os.path.basename(track_path))
    _kv("Spawn",       f"({sx:.2f}, {sy:.2f})  θ={stheta:.2f} rad")
    _kv("Checkpoints", str(n_cp))
    _kv("States",      "STRAIGHT → CORNER → SHARP | LOST")
    _rule()
    print()

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

    # Checkpoint state — sequential: next_cp is the index of the next required CP
    next_cp        = 0                 # index into checkpoints[]
    cp_hit_mask    = [False] * n_cp    # which CPs have been hit this lap
    lap_invalid    = False             # True if a wrong-order CP was hit this lap

    update_viz, fig, cp_circles, cp_rings = build_dashboard(
        track_arr, (sx, sy), checkpoints)

    RENDER_EVERY = 4
    step = 0
    t    = 0.0

    try:
        while t < MAX_TIME and plt.fignum_exists(fig.number):
            readings = read_sensors(robot, track_arr)
            weights  = readings ** 2
            ys       = (np.arange(N_SENSORS) - (N_SENSORS - 1) / 2) * SPACING_M
            total_w  = weights.sum()

            # Line position estimate
            if total_w > FSM_LOST_WEIGHT:
                e_y = last_ey = float(np.dot(weights, ys) / total_w)
            else:
                e_y = last_ey

            e_norm = e_y / HALF_SPAN

            # FSM → angular velocity command + target speed
            w_cmd, v_cmd, state = fsm.update(e_norm, total_w, DT)

            # Clamp steering so neither wheel reverses
            if state != State.LOST:
                w_max = 2.0 * v_cmd / WHEEL_BASE if v_cmd > 0 else 0.0
                w_cmd = float(np.clip(w_cmd, -w_max, w_max))

            vL_cmd = v_cmd - w_cmd * WHEEL_BASE / 2
            vR_cmd = v_cmd + w_cmd * WHEEL_BASE / 2

            robot.update(vL_cmd, vR_cmd)

            rx, ry = robot.pos

            # ── Checkpoint detection (sequential) ────────────────────────────
            if departed and not lap_invalid:
                for cp_idx, (cx, cy, cr) in enumerate(checkpoints):
                    if np.hypot(rx - cx, ry - cy) < cr:
                        if cp_idx == next_cp:
                            cp_hit_mask[next_cp] = True
                            _ok(f"CP{next_cp + 1}/{n_cp} at {t:.2f} s")
                            next_cp += 1
                        elif not cp_hit_mask[cp_idx]:
                            lap_invalid = True
                            _err(f"CP{cp_idx + 1} out of order  (expected CP{next_cp + 1})  — lap void")
                        break

            # ── Lap detection ────────────────────────────────────────────────
            dist_to_start = np.hypot(rx - sx, ry - sy)
            if not departed and dist_to_start > MIN_DEPART:
                departed    = True
                lap_start_t = t
            if departed and dist_to_start < START_RADIUS:
                if lap_invalid:
                    _err(f"Lap void  (wrong-order checkpoint)")
                elif next_cp == n_cp:
                    last_lap_time = t - lap_start_t
                    lap_count    += 1
                    _rule()
                    _ok(f"Lap {lap_count}  —  {last_lap_time:.2f} s")
                    _rule()
                else:
                    _warn(f"Finish crossed  —  only {next_cp}/{n_cp} CPs hit")
                next_cp     = 0
                cp_hit_mask = [False] * n_cp
                lap_invalid = False
                departed    = False

            if step % RENDER_EVERY == 0:
                update_viz(robot, readings, e_y, t,
                           lap_count, last_lap_time, state, cp_hit_mask, lap_invalid)

            t    += DT
            step += 1

    except KeyboardInterrupt:
        pass

    print()
    _rule()
    _ok(f"Simulation ended  —  t={t:.2f} s  |  {lap_count} lap(s)")
    _rule()
    print()
    plt.ioff()
    plt.show()


# ──────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Line-Following Robot Simulator")
    parser.add_argument("--track", default="assets/bane_fase2.png",
                        help="Path to track image (default: assets/bane_fase2.png)")
    parser.add_argument("--headless", "-H", action="store_true",
                        help="Run without GUI — console output only, much faster")
    parser.add_argument("--laps", "-n", type=int, default=0,
                        help="Stop after this many valid laps (headless only, 0 = use MAX_TIME)")

    # Tuner options
    parser.add_argument("--tune", "-T", action="store_true",
                        help="Run Uniform Random Search to find optimal PID/FSM params")
    parser.add_argument("--samples", type=int, default=500,
                        help="Number of random candidates to evaluate (default: 500)")
    parser.add_argument("--tune-time", type=float, default=60.0,
                        help="Simulated seconds per candidate (default: 60.0)")
    parser.add_argument("--top-k", type=int, default=5,
                        help="How many top results to display (default: 5)")
    parser.add_argument("--seed", type=int, default=0,
                        help="RNG seed for reproducibility (default: 0)")
    args = parser.parse_args()

    path = args.track
    if not os.path.isabs(path):
        path = os.path.join(os.path.dirname(__file__), path)

    if not os.path.exists(path):
        sys.exit(f"Error: track file not found: {path}")

    if args.tune:
        tune(path, n_samples=args.samples, tune_time=args.tune_time,
             top_k=args.top_k, seed=args.seed)
    elif args.headless:
        simulate_headless(path, stop_after_laps=args.laps)
    else:
        simulate(path)

