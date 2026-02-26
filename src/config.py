# config.py — single source of truth for all simulation parameters.
# Both main.py and lap_optimizer.py import from here.


# ── PID ───────────────────────────────────────────────────────────────────────
# PID receives normalised error: e_norm = e_y / QTR_HALF_SPAN  (range ±1).
# Gain conversion from physical robot:
#   SCALE = 4000 counts × (2 × 2.20/255 / 0.165) = 418.28
#   Kp_sim = Kp_phys × SCALE,  Kp_phys = Kp_sim / 418.28
QTR_HALF_SPAN = ((9 - 1) / 2) * 0.004   # 0.016 m — sensor array half-span

PID_KP    =  11.7   # rad/s per e_norm  (Kp_phys = 0.028)
PID_KI    =   0.042 # rad/(e_norm·s)    (Ki_phys = 0.0001)
PID_KD    =  50.1   # rad·s/e_norm      (Kd_phys = 0.1198)
PID_LIMIT =  23.5   # rad/s             (MAX_TURN = 225 PWM)

PID_INTEGRAL_LIMIT = 0.075  # norm·s — matched to physical: 300 counts / 4000
PID_DERIV_FILTER   = 0.30   # LP filter α on derivative term


# ── Speed controller ──────────────────────────────────────────────────────────
# SpeedController also receives e_norm (±1), not raw metres.
# turn_speed MUST be < straight_speed — robot slows in corners.
SC_STRAIGHT_SPEED        = 0.86   # m/s
SC_TURN_SPEED            = 0.50   # m/s — must be < straight_speed
SC_ERROR_THRESHOLD       = 0.25   # normalised error (0–1) that triggers TURNING mode
SC_SMOOTHING             = 0.08   # LP weight on new target speed (lower = smoother)
SC_TURN_FACTOR_THRESHOLD = 0.50   # fraction of pid_limit below which STRAIGHT is allowed
SC_MIN_SPEED_FACTOR      = 0.20   # floor speed multiplier at max steering


# ── Hardware ──────────────────────────────────────────────────────────────────
WHEEL_BASE      = 0.165   # m
WHEEL_RADIUS    = 0.017   # m
MAX_WHEEL_SPEED = 2.20    # m/s — no-load ceiling at 7.4V
MOTOR_TAU       = 0.060   # s  — first-order motor time constant
MOTOR_DEADZONE  = 0.33    # m/s — minimum speed that produces motion
ROBOT_MASS      = 0.245   # kg

COM_OFFSET_FROM_AXLE = 0.030   # m — CoM is 30 mm ahead of drive axle
ROBOT_MOI            = 0.00082 # kg·m² — yaw moment of inertia
WHEEL_LOAD_FRACTION  = 0.69    # fraction of weight on drive wheels
MU_SLIDE             = 1.14    # kinetic friction coefficient (sponge-rubber on vinyl)

MAX_LATERAL_ERROR = 0.05  # m — used in reporting only


# ── Sensor ────────────────────────────────────────────────────────────────────
# QTRX-HD-25RC array, 9 of 25 channels wired, RC mode, 4 mm pitch.
QTR_CHANNELS        = 9
QTR_SPACING_M       = 0.004   # m
QTR_SENSOR_OFFSET_M = 0.097   # m — sensor bar is 97 mm ahead of CoM
QTR_NOISE_STD       = 0.005   # Gaussian σ on normalised readings
QTR_ADC_BITS        = 10      # effective resolution (0–1000 per channel)

NOISE_SEED = 42               # fixed seed for reproducible runs


# ── Spawn positions ───────────────────────────────────────────────────────────
# x, y in metres (origin = bottom-left), theta in radians (0 = right, π/2 = up)
SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 2.00, "y": 0.14, "theta":  0.00},
    "suzuka.png":     {"x": 0.55, "y": 0.68, "theta": -0.40},
}


# ── Checkpoints ───────────────────────────────────────────────────────────────
# Robot must pass within CHECKPOINT_RADIUS of each point IN ORDER before a
# lap completion is accepted. Set list to [] to disable for a track.
CHECKPOINT_RADIUS = 0.10   # m

CHECKPOINT_REGISTRY = {
    "bane_fase2.png": [
        (2.80, 0.14),
        (3.80, 1.70),
        (2.00, 0.50),
        (0.20, 1.70),
    ],
    "suzuka.png": [
        (1.87, 0.68),
        (3.87, 1.30),
        (1.95, 1.45),
        (0.15, 1.05),
    ],
}


# ── Lap rules ─────────────────────────────────────────────────────────────────
START_FINISH_RADIUS = 0.10   # m — re-entry radius to finish a lap
MIN_DEPARTURE_DIST  = 0.30   # m — must leave start zone before finish counts
MAX_LAP_TIME        = 45.0   # s — hard DNF limit
MIN_LAP_TIME        = 5.0    # s — faster returns are ignored (false positives)
MAX_LINE_LOSS_TIME  = 0.5    # s — continuous line loss before DNF
LINE_THRESH         = 0.03   # minimum sensor weight to count as on-line


# ── Simulation ────────────────────────────────────────────────────────────────
# DT matched to physical ESP32 loop: 5× RC reads (~2.5 ms each) + PID ≈ 13 ms
DT           = 0.0125        # s — universal timestep (~80 Hz)
SIM_TIME     = 45.0          # s — hard cap on visual run
PX_PER_METER = 500           # pixels per metre
MAP_SIZE_M   = (4.0, 2.0)    # m — (width, height) of track area
