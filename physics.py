# physics.py — hardware constants derived from physical measurements.
# Do not edit unless you change the actual hardware.

ROBOT_MASS   = 0.270   # kg
WHEEL_BASE   = 0.180   # m   centre-to-centre
WHEEL_RADIUS = 0.017   # m   (34 mm diameter)
MAX_WHEEL    = 2.20    # m/s no-load at 7.4 V
MOTOR_TAU    = 0.055   # s   mechanical time constant
DEADZONE     = 0.04    # m/s minimum effective wheel speed

MOTOR_KE  = 0.0573  # V·s/rad
MOTOR_KT  = 0.0573  # N·m/A
MOTOR_R   = 13.5    # Ω
MOTOR_V   = 7.4     # V supply

N_SENSORS  = 8
SPACING_M  = 0.004   # m  4 mm pitch
SENSOR_FWD = 0.115   # m  sensor bar offset from CoM  (160 mm axle - 45 mm CoM)
NOISE_STD  = 0.003
HALF_SPAN  = ((N_SENSORS - 1) / 2) * SPACING_M   # 0.014 m

COM_OFFSET     = 0.045   # m  CoM ahead of drive axle
_AXLE_TO_FRONT = 0.160   # m  axle to front contact points
_COM_TO_FRONT  = _AXLE_TO_FRONT - COM_OFFSET
DRIVE_LOAD_REST = _COM_TO_FRONT / _AXLE_TO_FRONT   # ≈ 0.719

MU_LATERAL    = 1.06   # lateral friction coefficient (measured 1.12 × 0.95)
G             = 9.81   # m/s²
COM_HEIGHT    = 0.004  # m  (~half chassis height of 8 mm)
CONTACT_WIDTH = 0.022  # m  wheel contact patch width

# Yaw moment of inertia — inverted-T two-slab model
_M         = ROBOT_MASS
_Iz_cross  = (_M * 0.5) / 12.0 * (0.180**2 + 0.040**2)
_Iz_spine  = (_M * 0.5) / 12.0 * (0.160**2 + 0.040**2) + (_M * 0.5) * 0.035**2
ROBOT_IZ   = _Iz_cross + _Iz_spine   # ≈ 0.000854 kg·m²

