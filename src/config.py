# config.py


# ---- TRACK ----
PX_PER_METER = 500        # high resolution for realism
MAP_SIZE_M = (4.0, 2.0)   # width, height in meters

# ---- QTR-HD-25RC SENSOR ----
QTR_CHANNELS = 25
QTR_SPACING_M = 0.004     # 4 mm pitch  → array spans ±48 mm
QTR_SENSOR_OFFSET_M = 0.03  # 3 cm ahead of COM (reduced from 6cm for less lag)

QTR_NOISE_STD = 0.01      # reduced noise for cleaner readings
QTR_ADC_BITS = 12

# ---- ROBOT PHYSICS ----
WHEEL_BASE = 0.12         # distance between rear wheels
MAX_WHEEL_SPEED = 1.0

# Motor model
MOTOR_TAU = 0.05          # 50 ms — faster motor response

# COM & mass distribution
ROBOT_MASS = 0.9          # kg
MU_SLIDE = 1.14           # sliding friction coefficient

# ---- SIMULATION ----
DT = 0.005
SIM_TIME = 45.0   # upper limit — simulation stops early if lap is completed

# ---- LAP TIMER ----
# Shared by main.py and lap_optimizer.py — change here to affect both
START_FINISH_RADIUS = 0.10   # metres — robot must re-enter this to finish a lap
MIN_DEPARTURE_DIST  = 0.30   # metres — must leave start zone before finish counts
MAX_LAP_TIME        = 60.0   # seconds — DNF cutoff in optimizer

# A run is invalid if the robot loses the line for more than this many seconds
# (total sensor weight stays below LINE_THRESH continuously)
MAX_LINE_LOSS_TIME  = 1.0    # seconds — more than this = truly off track / DNF

# ---- PID + SPEED CONTROLLER DEFAULTS ----
# These are the parameters used by main.py for the visual run AND as the
# initial guess / centre point for the optimizer in lap_optimizer.py.
# Change values here — both programs will pick them up automatically.
PID_KP               = 120.0   # proportional gain
PID_KI               = 4.0     # integral gain
PID_KD               = 18.0    # derivative gain
PID_LIMIT            = 22.0    # output (angular velocity) clamp
PID_INTEGRAL_LIMIT   = 1.2     # windup guard
PID_DERIV_FILTER     = 0.10    # low-pass coefficient on derivative term

SC_STRAIGHT_SPEED    = 0.91    # m/s on straights
SC_TURN_SPEED        = 0.70    # m/s in corners
SC_ERROR_THRESHOLD   = 0.007   # lateral error that triggers speed reduction (m)
SC_SMOOTHING         = 0.12    # first-order speed blend coefficient

# ---- SENSOR NOISE SEED ----
# Fixed seed makes every simulation deterministic.
# main.py and lap_optimizer.py both set np.random.seed(NOISE_SEED) before running.
# Change to None for non-deterministic runs.
NOISE_SEED = 42

# ---- SPAWN REGISTRY ----
# Single source of truth for all spawn positions.
# Used by: main.py, lap_optimizer.py, preview_track.py
# x, y  : world coordinates in metres (origin = bottom-left)
# theta  : heading in radians (0 = pointing right, pi/2 = pointing up)
SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 2.00, "y": 0.14, "theta":  0.00},
    "suzuka.png":     {"x": 0.55, "y": 0.68, "theta": -0.40},
    # "my_track.png": {"x": 1.00, "y": 0.50, "theta":  0.00},  # add yours here
}

# Define MAX_LATERAL_ERROR as a global constant
MAX_LATERAL_ERROR = 0.05  # Maximum lateral error in meters (adjust as needed)

# ---- CHECKPOINT REGISTRY ----
# Checkpoints are (x, y) positions the robot must pass through (within
# CHECKPOINT_RADIUS metres) IN ORDER before a lap completion is counted.
# This prevents the optimizer from "teleporting" past sections of the track.
# Add one entry per track; leave empty list [] to disable checkpoints for a track.
#
# How to find good checkpoint positions:
#   Run main.py once with --track and note the robot's path.
#   Pick evenly-spaced waypoints around the circuit.
#   Four checkpoints (at ~25 %, 50 %, 75 %, 100 % of the circuit) is enough.
#
CHECKPOINT_RADIUS = 0.10   # metres — how close the robot must get to each checkpoint

CHECKPOINT_REGISTRY = {
    # bane_fase2.png — four checkpoints distributed around the track
    "bane_fase2.png": [
        (2.80, 0.14),   # far end of bottom straight
        (3.80, 1.70),   # top right corner
        (2.00, 0.50),   # bottom of box
        (0.20, 1.70),   # top left corner
    ],
    # suzuka.png — four checkpoints distributed around the circuit
    "suzuka.png": [
        (1.87, 0.68),   # after the first chicane
        (3.87, 1.30),   # mid-sector 2
        (1.95, 1.45),   # top of the circuit
        (0.15, 1.05),   # return section
    ],
}

