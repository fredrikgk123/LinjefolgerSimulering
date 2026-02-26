# =============================================================================
#  config.py  —  SINGLE SOURCE OF TRUTH FOR ALL PARAMETERS
# =============================================================================
#  Every parameter the robot, sensor, PID controller, speed controller,
#  optimizer, and track system use lives here.
#
#  Both  main.py  and  lap_optimizer.py  import from this file, so any
#  value you change here is instantly reflected in both programs — you
#  never have to edit two files.
#
#  QUICK-FIND INDEX
#  ─────────────────────────────────────────────────────────────────────
#  Section                         | Jump to tag
#  ─────────────────────────────────────────────────────────────────────
#  1. PID Controller               | §PID
#  2. Speed Controller             | §SPEED
#  3. Robot Hardware               | §HARDWARE
#  4. Sensor (QTR-HD-25RC)         | §SENSOR
#  5. Spawn Positions              | §SPAWN
#  6. Checkpoints                  | §CHECKPOINTS
#  7. Lap-Timer / DNF Rules        | §LAP
#  8. Simulation Settings          | §SIM
#  ─────────────────────────────────────────────────────────────────────
# =============================================================================


# =============================================================================
#  §PID — PID CONTROLLER
# =============================================================================
#  These are the gains for the line-following PID loop.
#  The optimizer in lap_optimizer.py uses them as its initial guess, so
#  paste improved values from the optimizer output back here to make the
#  next run start from the better baseline.
#
#  KP  — how aggressively the robot reacts to current error
#  KI  — corrects accumulated (integral) error; keep small to avoid windup
#  KD  — damps oscillation by reacting to the rate of change of error
# =============================================================================

PID_KP             = 122.0   # proportional gain
PID_KI             = 4.07    # integral gain
PID_KD             = 19.3    # derivative gain
PID_LIMIT          = 18.0    # max angular-velocity command (rad/s); clamps PID output
PID_INTEGRAL_LIMIT = 1.2     # anti-windup clamp on the integral accumulator
PID_DERIV_FILTER   = 0.10    # low-pass coefficient on the derivative term (0–1)


# =============================================================================
#  §SPEED — SPEED CONTROLLER
# =============================================================================
#  The speed controller slows the robot in corners and speeds it up on
#  straights, based on the current lateral error.
#
#  SC_STRAIGHT_SPEED   — target wheel speed when the track is straight
#  SC_TURN_SPEED       — target wheel speed when cornering
#  SC_ERROR_THRESHOLD  — lateral error (m) above which "corner" mode kicks in
#  SC_SMOOTHING        — blending coefficient between speed states (0 = instant,
#                        1 = never changes); lower = snappier transitions
# =============================================================================

SC_STRAIGHT_SPEED  = 1.097 # m/s on straights
SC_TURN_SPEED      = 0.960   # m/s in corners
SC_ERROR_THRESHOLD = 0.0020  # lateral error (m) that triggers speed reduction
SC_SMOOTHING       = 0.300   # first-order speed blend coefficient


# =============================================================================
#  §HARDWARE — ROBOT HARDWARE / PHYSICAL PROPERTIES
# =============================================================================
#  Change these to match your real robot's specifications.
# =============================================================================

WHEEL_BASE      = 0.12   # metres — distance between the two drive wheels
MAX_WHEEL_SPEED = 1.0    # m/s   — motor speed limit (hardware maximum)

MOTOR_TAU       = 0.05   # seconds — motor time-constant (50 ms = faster response)

ROBOT_MASS      = 0.9    # kg
MU_SLIDE        = 1.14   # sliding (kinetic) friction coefficient of the tyres

MAX_LATERAL_ERROR = 0.05  # metres — maximum lateral error before considered off-track


# =============================================================================
#  §SENSOR — QTR-HD-25RC SENSOR ARRAY
# =============================================================================
#  Simulation of the Pololu QTR-HD-25RC reflectance sensor array.
#  Adjust if you swap to a different sensor or change its mounting position.
# =============================================================================

QTR_CHANNELS       = 25      # number of sensor channels
QTR_SPACING_M      = 0.004   # metres — 4 mm pitch; total array span ≈ ±48 mm
QTR_SENSOR_OFFSET_M = 0.03   # metres — sensor is 3 cm ahead of the centre of mass
QTR_NOISE_STD      = 0.01    # standard deviation of Gaussian noise on readings
QTR_ADC_BITS       = 12      # ADC resolution (12-bit = 0–4095 counts)

# Fixed random seed — keeps every simulation run deterministic so results are
# reproducible. Change to None to get different noise on every run.
NOISE_SEED = 42


# =============================================================================
#  §SPAWN — SPAWN POSITIONS PER TRACK
# =============================================================================
#  The robot's starting position and heading for each track image.
#  Used by: main.py, lap_optimizer.py, preview_track.py
#
#  x, y  : world coordinates in metres (origin = bottom-left corner of image)
#  theta : heading in radians  (0 = pointing right,  π/2 ≈ 1.57 = pointing up)
#
#  To add a new track, copy one of the existing entries and adjust the values.
# =============================================================================

SPAWN_REGISTRY = {
    "bane_fase2.png": {"x": 2.00, "y": 0.14, "theta":  0.00},
    "suzuka.png":     {"x": 0.55, "y": 0.68, "theta": -0.40},
    # "my_track.png": {"x": 1.00, "y": 0.50, "theta":  0.00},  # ← add yours here
}


# =============================================================================
#  §CHECKPOINTS — CHECKPOINT POSITIONS PER TRACK
# =============================================================================
#  Checkpoints force the robot to complete the whole circuit in order.
#  The robot must pass within CHECKPOINT_RADIUS metres of each (x, y) point
#  IN ORDER before a lap completion is counted. This stops the optimizer from
#  finding "cheating" shortcuts.
#
#  How to find good positions:
#    1. Run  main.py  and watch the robot's path on the live map.
#    2. Pick four evenly-spaced waypoints (roughly 25 %, 50 %, 75 %, 100 %).
#    3. Yellow circles in the live view show where each checkpoint sits —
#       adjust x/y until they clearly lie on the track line.
#
#  To DISABLE checkpoints for a track, set its list to [].
# =============================================================================

CHECKPOINT_RADIUS = 0.10   # metres — how close the robot must get to clear a checkpoint

CHECKPOINT_REGISTRY = {
    # --- bane_fase2.png ---
    "bane_fase2.png": [
        (2.80, 0.14),   # 25 % — far end of bottom straight
        (3.80, 1.70),   # 50 % — top right corner
        (2.00, 0.50),   # 75 % — bottom of box
        (0.20, 1.70),   # 100% — top left corner
    ],
    # --- suzuka.png ---
    "suzuka.png": [
        (1.87, 0.68),   # 25 % — after the first chicane
        (3.87, 1.30),   # 50 % — mid-sector 2
        (1.95, 1.45),   # 75 % — top of the circuit
        (0.15, 1.05),   # 100% — return section
    ],
    # "my_track.png": [(x1, y1), (x2, y2), (x3, y3), (x4, y4)],  # ← add yours here
}


# =============================================================================
#  §LAP — LAP-TIMER / DNF RULES
# =============================================================================
#  These thresholds control when a lap is counted as valid or as a DNF (Did
#  Not Finish). Shared between main.py and lap_optimizer.py.
# =============================================================================

START_FINISH_RADIUS = 0.10   # metres — robot must re-enter this circle to finish a lap
MIN_DEPARTURE_DIST  = 0.30   # metres — must leave start zone before finish counts
MAX_LAP_TIME        = 45.0   # seconds — any lap slower than this is a DNF (matches SIM_TIME)
MAX_LINE_LOSS_TIME  = 0.3    # seconds — continuous line-loss longer than this = DNF
LINE_THRESH         = 0.08   # minimum total sensor weight to count as "on line"
                             # (used identically by main.py and lap_optimizer.py)


# =============================================================================
#  §SIM — SIMULATION SETTINGS
# =============================================================================
#  Low-level simulation parameters. You rarely need to change these unless
#  you are experimenting with numerical stability or map resolution.
#
#  IMPORTANT: DT is the UNIVERSAL timestep used by BOTH the optimizer and
#  visualization. Changing this value will affect PID behavior, so you may
#  need to re-tune PID gains if you change it significantly.
# =============================================================================

DT          = 0.005        # seconds — UNIVERSAL physics timestep (200 Hz)
                           # Used by both optimizer and visualization
                           # Increase for faster simulation, decrease for accuracy
SIM_TIME    = 45.0         # seconds — hard cap on visual-run duration
PX_PER_METER = 500         # pixels per metre — image resolution scale
MAP_SIZE_M  = (4.0, 2.0)   # metres  — (width, height) of the track area

