#!/usr/bin/env python3
"""
Debug script to test sensor readings at the starting position.
"""

import numpy as np
from PIL import ImageFilter, Image
from track.track_generator import generate_default_track
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot, world_to_pixel, bilinear_sample
from config import *

def main():
    print("="*70)
    print("SENSOR DEBUG TEST")
    print("="*70)

    # Generate track
    track = generate_default_track()
    blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    print(f"\n📊 Track Info:")
    print(f"  Track image size: {blur_arr.shape}")
    print(f"  World size: {MAP_SIZE_M} m")
    print(f"  Pixels per meter: {PX_PER_METER}")

    # Create robot at starting position (from main.py)
    W, H = MAP_SIZE_M
    x_start = 0.05 * W  # 0.15m
    y_start = 0.5 * H + 0.25 * H * np.sin(3 * x_start / W * 2 * np.pi)  # Actual track position

    omega = 3.0 * 2 * np.pi / W
    slope = 0.25 * H * omega * np.cos(omega * x_start)
    theta_start = np.arctan(slope)

    robot = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta = theta_start

    print(f"\n🤖 Robot Starting Position:")
    print(f"  Position (world): ({robot.position[0]:.3f}, {robot.position[1]:.3f}) m")
    print(f"  Heading: {np.degrees(robot.theta):.1f}°")

    # Convert to pixels
    px, py = world_to_pixel(robot.position[0], robot.position[1])
    print(f"  Position (pixels): ({px:.1f}, {py:.1f})")
    print(f"  Image value at robot: {blur_arr[int(py), int(px)]:.1f}")

    # Check where the track actually is
    print(f"\n🛤️  Track Position Check:")
    track_y_world = 0.5 * H + 0.25 * H * np.sin(3 * x_start / W * 2 * np.pi)
    print(f"  Track Y at x={x_start:.3f}m: {track_y_world:.3f}m")
    print(f"  Robot Y: {y_start:.3f}m")
    print(f"  Difference: {abs(track_y_world - y_start)*1000:.1f}mm")

    if abs(track_y_world - y_start) > 0.01:
        print(f"  ⚠️  WARNING: Robot is {abs(track_y_world - y_start)*1000:.0f}mm off the track!")

    # Sample the track line
    print(f"\n📍 Track Line Sampling:")
    for y_test in np.linspace(0.1*H, 0.9*H, 9):
        px_test, py_test = world_to_pixel(x_start, y_test)
        val = bilinear_sample(blur_arr, px_test, py_test)
        marker = "  ← DARK (LINE)" if val < 100 else ""
        print(f"  y={y_test:.3f}m → pixel value: {val:6.1f}{marker}")

    # Test sensors
    sensors = QTRArray()
    readings = sensors.read(robot, blur_arr)

    print(f"\n📡 Sensor Readings:")
    print(f"  Sensor offset: {QTR_SENSOR_OFFSET_M*100:.1f}cm ahead of robot")
    print(f"  Number of sensors: {QTR_CHANNELS}")
    print(f"  Sensor spacing: {QTR_SPACING_M*1000:.1f}mm")

    # Show sensor positions in world coordinates
    R = robot.rotation_matrix()
    sensor_xy_world = (R @ sensors.sensor_pos_body.T).T + robot.position

    print(f"\n  Sensor array center position:")
    center_idx = QTR_CHANNELS // 2
    sx, sy = sensor_xy_world[center_idx]
    print(f"    World: ({sx:.3f}, {sy:.3f}) m")
    px, py = world_to_pixel(sx, sy)
    print(f"    Pixels: ({px:.1f}, {py:.1f})")
    print(f"    Image value: {blur_arr[int(py), int(px)]:.1f}")

    print(f"\n  Individual sensor readings:")
    total_reading = 0
    on_sensors = 0
    for i, reading in enumerate(readings):
        sx, sy = sensor_xy_world[i]
        lateral_pos = sensors.sensor_pos_body[i, 1]
        px, py = world_to_pixel(sx, sy)
        img_val = blur_arr[int(py), int(px)]

        status = ""
        if reading > 0.5:
            status = " ✓ ON LINE"
            on_sensors += 1
        elif reading > 0.15:
            status = " ~ edge"

        total_reading += reading

        if i % 5 == 0 or reading > 0.15:  # Show every 5th sensor or active ones
            print(f"    [{i:2d}] lateral={lateral_pos*1000:+5.1f}mm → "
                  f"reading={reading:.3f} (img={img_val:5.1f}){status}")

    print(f"\n📊 Summary:")
    print(f"  Total sensor activation: {total_reading:.3f}")
    print(f"  Sensors on line: {on_sensors}/{QTR_CHANNELS}")
    print(f"  Weighted sum: {(readings**2).sum():.3f}")

    # Compute centroid error
    weights = readings ** 2
    pos_y = sensors.sensor_pos_body[:, 1]
    total_w = weights.sum()

    if total_w > 0.08:
        e_y = float(np.dot(weights, pos_y) / total_w)
        print(f"  Lateral error: {e_y*1000:.1f}mm")
        print(f"  ✅ Line is detected!")
    else:
        print(f"  ❌ Line NOT detected (total weight {total_w:.3f} < 0.08)")
        print(f"\n🔍 Diagnosis:")
        print(f"    1. Robot might be off the track")
        print(f"    2. Sensor offset might be wrong")
        print(f"    3. Track might not exist at this position")

    # Save a debug image showing robot and sensor positions
    print(f"\n💾 Saving debug visualization...")
    debug_img = Image.fromarray(blur_arr.astype(np.uint8))
    debug_img = debug_img.convert('RGB')
    from PIL import ImageDraw
    draw = ImageDraw.Draw(debug_img)

    # Draw robot position
    rpx, rpy = world_to_pixel(robot.position[0], robot.position[1])
    draw.ellipse([rpx-5, rpy-5, rpx+5, rpy+5], outline=(255,0,0), width=2)

    # Draw sensor positions
    for sx, sy in sensor_xy_world:
        spx, spy = world_to_pixel(sx, sy)
        draw.ellipse([spx-2, spy-2, spx+2, spy+2], fill=(0,255,0))

    debug_img.save('tests/debug_sensor_positions.png')
    print(f"  Saved: tests/debug_sensor_positions.png")

    print("="*70)

if __name__ == "__main__":
    main()

