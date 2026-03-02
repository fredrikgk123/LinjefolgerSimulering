# config.py — all tunable simulation parameters.
# Edit freely. Physical hardware values live in physics.py.

# ── Simulation ────────────────────────────────────────────────────────────────
DT       = 0.005   # timestep (s) — 200 Hz, matches ESP32 loop
MAX_TIME = 80.0    # hard stop (s)
PX_PER_M = 500     # pixels per metre
MAP_W_M  = 4.0     # track image width  (m)
MAP_H_M  = 2.0     # track image height (m)

# ── PID gains per FSM state ───────────────────────────────────────────────────
# Each tuple: (kp, ki, kd, limit, speed_m_s)
# limit = max |ω_cmd| in rad/s; speed = target forward speed in m/s
PID_STRAIGHT = (0.8,  0.0,  0.35,  5.0,  0.90)
PID_CORNER   = (1.4,  0.0,  0.70,  5.0,  0.60)
PID_SHARP    = (2.0,  0.0,  1.10,  5.0,  0.35)
PID_LOST     = (0.0,  0.0,  0.00,  5.0,  0.10)

INTEG_LIMIT  = 0.06   # anti-windup clamp on integral (keep small while KI = 0)
DERIV_ALPHA  = 0.35   # EMA weight on derivative  (0 = frozen, 1 = raw)
ACCEL_RATE   = 15.0   # speed ramp  (m/s²)

# ── FSM state-change thresholds (normalised error 0–1) ───────────────────────
FSM_STR_EXIT    = 0.40   # STRAIGHT → CORNER
FSM_STR_ENTER   = 0.20   # CORNER   → STRAIGHT
FSM_SHARP_ENTER = 0.80   # CORNER   → SHARP
FSM_SHARP_EXIT  = 0.65   # SHARP    → CORNER  (hysteresis)
FSM_LOST_WEIGHT  = 0.002  # total sensor weight below this → LOST
FSM_FOUND_WEIGHT = 0.015  # weight above this → leave LOST
FSM_LOST_SEARCH_W = 1.2  # yaw rate while searching (rad/s)

# ── Track layout ──────────────────────────────────────────────────────────────
# Spawn: (x, y) metres from bottom-left, theta radians
SPAWNS = {
    "bane_fase2.png": (2.00, 0.14,  0.00),
    "suzuka.png":     (0.55, 0.68, -0.40),
}

# Checkpoints: ordered list of (cx, cy, radius) in metres.
# All must be hit in order for a lap to count.
CHECKPOINTS = {
    "bane_fase2.png": [
        (3.30, 0.14, 0.12),
        (3.35, 1.75, 0.12),
        (3.75, 1.20, 0.12),
        (2.00, 0.50, 0.12),
        (0.25, 1.20, 0.12),
        (0.65, 1.75, 0.12),
        (0.70, 0.14, 0.12),
    ],
    "suzuka.png": [
        (2.00, 0.68, 0.12),
        (3.20, 1.20, 0.12),
        (2.00, 1.60, 0.12),
        (0.80, 1.20, 0.12),
    ],
}

# ── Tuner search space ────────────────────────────────────────────────────────
# Each entry: (low, high) — the tuner samples uniformly from this range.
# Narrow a range around a known-good value for a fine search.
# Never set low == high.  Constraints (kd/kp ratio, speed ordering) are
# enforced automatically after sampling.
SEARCH_SPACE = {
    # STRAIGHT
    "str_kp":    (0.3,  10.0),
    "str_kd":    (0.1,  10.0),
    "str_limit": (2.0,  10.0),
    "str_spd":   (0.5,  5.8),
    # CORNER
    "cor_kp":    (0.5,  10.0),
    "cor_kd":    (0.2,  10.0),
    "cor_limit": (2.0,  10.0),
    "cor_spd":   (0.3,  4.2),
    # SHARP
    "shp_kp":    (1.0,  10.0),
    "shp_kd":    (0.5,  10.0),
    "shp_limit": (2.0,  10.0),
    "shp_spd":   (0.15, 3.7),
    # FSM thresholds
    "str_exit":    (0.20, 0.60),
    "str_enter":   (0.05, 0.35),
    "shp_enter":   (0.60, 0.95),
    "shp_exit":    (0.45, 0.85),
    # Misc
    "deriv_alpha":   (0.10, 0.60),
    "lost_search_w": (0.5,  2.5),
    "accel_rate":    (5.0,  40.0),
}

