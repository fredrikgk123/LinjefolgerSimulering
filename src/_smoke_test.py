#!/usr/bin/env python3
"""Smoke test: run one full simulated step with all new physics."""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
np.random.seed(42)

import config
from physics.friction import SLIP_INFLUENCE, friction_force, slip_ratio
from physics.robot_model import Robot, _apply_deadzone, world_to_pixel
from sensors.qtr_array import QTRArray

# ── Config checks ──────────────────────────────────────────────────────────
assert hasattr(config, 'ROBOT_MOI'),              "ROBOT_MOI missing"
assert hasattr(config, 'COM_OFFSET_FROM_AXLE'),   "COM_OFFSET_FROM_AXLE missing"
assert hasattr(config, 'WHEEL_LOAD_FRACTION'),     "WHEEL_LOAD_FRACTION missing"
assert hasattr(config, 'MOTOR_DEADZONE'),          "MOTOR_DEADZONE missing"
assert config.MOTOR_TAU   == 0.060,  f"MOTOR_TAU={config.MOTOR_TAU}"
assert config.SLIP_INFLUENCE_VAL if False else SLIP_INFLUENCE == 0.35, \
    f"SLIP_INFLUENCE={SLIP_INFLUENCE}"
assert config.ROBOT_MOI   == 0.00082, f"ROBOT_MOI={config.ROBOT_MOI}"
assert config.WHEEL_LOAD_FRACTION == 0.69, f"WLF={config.WHEEL_LOAD_FRACTION}"
assert config.MOTOR_DEADZONE == 0.33, f"DZ={config.MOTOR_DEADZONE}"

# ── Dead zone ──────────────────────────────────────────────────────────────
assert _apply_deadzone(0.10) == 0.0,  "deadzone: 0.10 should be 0"
assert _apply_deadzone(0.33) == 0.0,  "deadzone: threshold should be 0"
assert _apply_deadzone(0.50) > 0.0,   "deadzone: 0.50 should pass"
assert _apply_deadzone(-0.50) < 0.0,  "deadzone: negative should pass"
print("Dead zone: OK")

# ── Robot update — check omega state is tracked ────────────────────────────
robot = Robot()
robot.vL = 0.0; robot.vR = 0.0; robot.omega = 0.0

# Apply a strong turn command
for _ in range(10):
    robot.update(vL_cmd=0.5, vR_cmd=1.5)

assert robot.omega != 0.0, "omega should be non-zero after turning"
assert robot.omega < 15.0, "omega shouldn't explode"
print(f"Rotational inertia: omega={robot.omega:.3f} rad/s after 10 steps  OK")

# ── Dead zone prevents creep ───────────────────────────────────────────────
robot2 = Robot()
for _ in range(50):
    robot2.update(0.10, 0.10)  # below deadzone
assert robot2.vL < 0.01 and robot2.vR < 0.01, "Below deadzone should not move"
print(f"Deadzone creep prevention: vL={robot2.vL:.4f} vR={robot2.vR:.4f}  OK")

# ── Sensor array ──────────────────────────────────────────────────────────
q = QTRArray()
assert q.sensor_pos_body.shape == (9, 2)
print("QTRArray(9): OK")

# ── Summary ────────────────────────────────────────────────────────────────
print()
print("=" * 55)
print("  PHYSICS SMOKE TEST PASSED")
print("=" * 55)
print(f"  MOTOR_TAU        = {config.MOTOR_TAU*1000:.0f} ms")
print(f"  ROBOT_MOI        = {config.ROBOT_MOI:.5f} kg·m²")
print(f"  WHEEL_LOAD_FRAC  = {config.WHEEL_LOAD_FRACTION:.2f}  (69% weight on wheels)")
print(f"  MOTOR_DEADZONE   = {config.MOTOR_DEADZONE:.2f} m/s")
print(f"  SLIP_INFLUENCE   = {SLIP_INFLUENCE:.2f}")
print(f"  COM_OFFSET       = {config.COM_OFFSET_FROM_AXLE*1000:.0f} mm forward of axle")
print("=" * 55)

