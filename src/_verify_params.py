#!/usr/bin/env python3
"""Quick sanity-check: verify all physical parameters loaded correctly."""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import config

assert config.QTR_CHANNELS == 9,         f"Expected 9, got {config.QTR_CHANNELS}"
assert config.QTR_SPACING_M == 0.004,    f"Expected 0.004, got {config.QTR_SPACING_M}"
assert config.QTR_NOISE_STD == 0.005,    f"Expected 0.005, got {config.QTR_NOISE_STD}"
assert config.QTR_ADC_BITS == 10,        f"Expected 10, got {config.QTR_ADC_BITS}"
assert config.MOTOR_TAU == 0.030,        f"Expected 0.030, got {config.MOTOR_TAU}"
assert config.MAX_WHEEL_SPEED == 2.20,   f"Expected 2.20, got {config.MAX_WHEEL_SPEED}"
assert config.WHEEL_BASE == 0.165,       f"Expected 0.165, got {config.WHEEL_BASE}"
assert config.WHEEL_RADIUS == 0.017,     f"Expected 0.017, got {config.WHEEL_RADIUS}"
assert config.ROBOT_MASS == 0.245,       f"Expected 0.245, got {config.ROBOT_MASS}"
assert config.MU_SLIDE == 1.14,          f"Expected 1.14, got {config.MU_SLIDE}"
assert config.DT == 0.0125,              f"Expected 0.0125, got {config.DT}"
assert config.LINE_THRESH == 0.03,       f"Expected 0.03, got {config.LINE_THRESH}"

from physics.friction import SLIP_INFLUENCE, slip_ratio, friction_force
assert SLIP_INFLUENCE == 0.15,           f"Expected 0.15, got {SLIP_INFLUENCE}"

from physics.robot_model import Robot, world_to_pixel, bilinear_sample
r = Robot()
assert r.mass == 0.245

from sensors.qtr_array import QTRArray
q = QTRArray()
assert q.sensor_pos_body.shape == (9, 2), f"Expected (9,2), got {q.sensor_pos_body.shape}"

print("=" * 50)
print("  ALL PHYSICAL PARAMETERS VERIFIED OK")
print("=" * 50)
print(f"  QTR_CHANNELS        : {config.QTR_CHANNELS}")
print(f"  QTR_SPACING_M       : {config.QTR_SPACING_M*1000:.1f} mm")
print(f"  QTR_SENSOR_OFFSET_M : {config.QTR_SENSOR_OFFSET_M*1000:.0f} mm")
print(f"  QTR_NOISE_STD       : {config.QTR_NOISE_STD}")
print(f"  QTR_ADC_BITS        : {config.QTR_ADC_BITS}")
print(f"  MOTOR_TAU           : {config.MOTOR_TAU*1000:.0f} ms")
print(f"  MAX_WHEEL_SPEED     : {config.MAX_WHEEL_SPEED:.2f} m/s")
print(f"  WHEEL_BASE          : {config.WHEEL_BASE*1000:.0f} mm")
print(f"  WHEEL_RADIUS        : {config.WHEEL_RADIUS*1000:.0f} mm")
print(f"  ROBOT_MASS          : {config.ROBOT_MASS*1000:.0f} g")
print(f"  MU_SLIDE            : {config.MU_SLIDE}")
print(f"  SLIP_INFLUENCE      : {SLIP_INFLUENCE}")
print(f"  DT                  : {config.DT*1000:.1f} ms  (~{1/config.DT:.0f} Hz)")
print(f"  LINE_THRESH         : {config.LINE_THRESH}")
print(f"  Sensor array span   : {(config.QTR_CHANNELS-1)*config.QTR_SPACING_M*1000:.0f} mm")
print("=" * 50)

