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
SIM_TIME = 30.0
