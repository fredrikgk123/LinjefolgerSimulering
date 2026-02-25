# config.py

import numpy as np

# ---- TRACK ----
TRACK_WIDTH_M = 0.020     # 20 mm — competition standard (increased from 15mm)
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
WHEEL_RADIUS = 0.03
MAX_WHEEL_SPEED = 1.0

# Motor model
MOTOR_TAU = 0.05          # 50 ms — faster motor response

# COM & mass distribution
ROBOT_MASS = 0.9          # kg
COM_OFFSET = np.array([0.02, 0.0])  # forward shift of COM
MU_SLIDE = 1.14           # sliding friction coefficient
MU_ROLL = 0.02            # rolling resistance

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

# ---- SENSOR NOISE SEED ----
# Fixed seed makes every simulation deterministic.
# main.py and lap_optimizer.py both set np.random.seed(NOISE_SEED) before running.
# Change to None for non-deterministic runs.
NOISE_SEED = 42

# ---- SPAWN REGISTRY ----
# Single source of truth for all spawn positions.
# Used by: main.py, multi_track_simulator.py, lap_optimizer.py
# x, y  : world coordinates in metres (origin = bottom-left)
# theta  : heading in radians (0 = pointing right, pi/2 = pointing up)
SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 3.00, "y": 0.14, "theta":  0.00},
    "suzuka.png":     {"x": 0.55, "y": 0.68, "theta": -0.40},
    # "my_track.png": {"x": 1.00, "y": 0.50, "theta":  0.00},  # add yours here
}

