#!/usr/bin/env python3
"""
Test that sensors continue to detect the line throughout the simulation.
"""

import numpy as np
from PIL import ImageFilter
from track.track_generator import generate_default_track
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot
from config import *

def main():
    print("="*70)
    print("SENSOR TRACKING TEST")
    print("="*70)

    # Generate track
    track = generate_default_track()
    blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    # Create robot at starting position
    W, H = MAP_SIZE_M
    x_start = 0.05 * W
    y_start = 0.5 * H + 0.25 * H * np.sin(3 * x_start / W * 2 * np.pi)

    omega = 3.0 * 2 * np.pi / W
    slope = 0.25 * H * omega * np.cos(omega * x_start)
    theta_start = np.arctan(slope)

    robot = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta = theta_start

    sensors = QTRArray()

    print(f"\nTesting sensor readings as robot moves along the track...")
    print(f"Starting at: ({x_start:.3f}, {y_start:.3f}) m, heading: {np.degrees(theta_start):.1f}°\n")

    # Simulate a few steps with no control (just move forward)
    test_steps = 50
    v_test = 0.3  # m/s

    for step in range(test_steps):
        # Read sensors
        readings = sensors.read(robot, blur_arr)

        # Compute detection metrics
        weights = readings ** 2
        pos_y = sensors.sensor_pos_body[:, 1]
        total_w = weights.sum()

        on_line = total_w > 0.08

        if total_w > 0.08:
            e_y = float(np.dot(weights, pos_y) / total_w)
        else:
            e_y = 0.0

        # Count sensors detecting line
        on_sensors = np.sum(readings > 0.5)

        t = step * DT

        if step % 10 == 0:  # Print every 10 steps
            status = "✓" if on_line else "✗"
            print(f"  t={t:5.3f}s  pos=({robot.position[0]:.3f}, {robot.position[1]:.3f})  "
                  f"θ={np.degrees(robot.theta):6.1f}°  sensors_on={on_sensors:2d}  "
                  f"total_w={total_w:5.2f}  e_y={e_y*1000:+6.1f}mm  {status}")

        # Move robot forward
        robot.update(v_test, v_test)

    print(f"\n✅ Sensor test complete!")
    print(f"   Final position: ({robot.position[0]:.3f}, {robot.position[1]:.3f}) m")
    print(f"   Distance traveled: {robot.position[0] - x_start:.3f} m")

    print("="*70)

if __name__ == "__main__":
    main()

