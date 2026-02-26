#!/usr/bin/env python3
"""
Debug script to test sensor readings at the starting position.
Uses the actual track images (bane_fase2.png / suzuka.png).
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from PIL import ImageFilter, Image
from track.image_loader import load_track_image
from sensors.qtr_array import QTRArray
from physics.robot_model import Robot, world_to_pixel, bilinear_sample
from config import MAP_SIZE_M, PX_PER_METER, QTR_SENSOR_OFFSET_M, QTR_CHANNELS, QTR_SPACING_M, SPAWN_REGISTRY

_HERE       = os.path.dirname(__file__)
_ASSETS_DIR = os.path.normpath(os.path.join(_HERE, '..', '..', 'assets'))

TRACK_FILENAME = "bane_fase2.png"   # change to "suzuka.png" to debug that track


def main():
    print("="*70)
    print("SENSOR DEBUG TEST")
    print("="*70)

    # Load track image
    track_path = os.path.join(_ASSETS_DIR, TRACK_FILENAME)
    track   = load_track_image(track_path)
    blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
    blur_arr = np.array(blurred, dtype=np.float32)

    W, H = MAP_SIZE_M
    print(f"\n📊 Track Info:")
    print(f"  Track image size: {blur_arr.shape}")
    print(f"  World size: {MAP_SIZE_M} m")
    print(f"  Pixels per meter: {PX_PER_METER}")

    # Use spawn registry for exact start position
    if TRACK_FILENAME not in SPAWN_REGISTRY:
        print(f"[warn] No spawn entry for '{TRACK_FILENAME}' — using centre.")
        sp = {"x": W / 2, "y": H / 2, "theta": 0.0}
    else:
        sp = SPAWN_REGISTRY[TRACK_FILENAME]

    x_start, y_start, theta_start = sp["x"], sp["y"], sp["theta"]

    robot = Robot()
    robot.position = np.array([x_start, y_start])
    robot.theta    = theta_start

    print(f"\n🤖 Robot Starting Position:")
    print(f"  Position (world): ({robot.position[0]:.3f}, {robot.position[1]:.3f}) m")
    print(f"  Heading: {np.degrees(robot.theta):.1f}°")

    px, py = world_to_pixel(robot.position[0], robot.position[1])
    print(f"  Position (pixels): ({px:.1f}, {py:.1f})")
    print(f"  Image value at robot: {blur_arr[int(py), int(px)]:.1f}")

    # Sample the track line around the spawn x
    print(f"\n📍 Track Line Sampling (at x={x_start:.3f} m):")
    for y_test in np.linspace(0.05*H, 0.95*H, 9):
        px_test, py_test = world_to_pixel(x_start, y_test)
        val = bilinear_sample(blur_arr, px_test, py_test)
        marker = "  ← DARK (LINE)" if val < 100 else ""
        print(f"  y={y_test:.3f}m → pixel value: {val:6.1f}{marker}")

    # Test sensors
    sensors  = QTRArray()
    readings = sensors.read(robot, blur_arr)

    print(f"\n📡 Sensor Readings:")
    print(f"  Sensor offset: {QTR_SENSOR_OFFSET_M*100:.1f}cm ahead of robot")
    print(f"  Number of sensors: {QTR_CHANNELS}")
    print(f"  Sensor spacing: {QTR_SPACING_M*1000:.1f}mm")

    R = robot.rotation_matrix()
    sensor_xy_world = (R @ sensors.sensor_pos_body.T).T + robot.position

    print(f"\n  Sensor array centre position:")
    center_idx = QTR_CHANNELS // 2
    sx, sy = sensor_xy_world[center_idx]
    print(f"    World: ({sx:.3f}, {sy:.3f}) m")
    cpx, cpy = world_to_pixel(sx, sy)
    print(f"    Pixels: ({cpx:.1f}, {cpy:.1f})")
    print(f"    Image value: {blur_arr[int(cpy), int(cpx)]:.1f}")

    print(f"\n  Individual sensor readings:")
    total_reading = 0
    on_sensors    = 0
    for i, reading in enumerate(readings):
        sx, sy       = sensor_xy_world[i]
        lateral_pos  = sensors.sensor_pos_body[i, 1]
        spx, spy     = world_to_pixel(sx, sy)
        img_val      = blur_arr[int(spy), int(spx)]

        status = ""
        if reading > 0.5:
            status = " ✓ ON LINE"
            on_sensors += 1
        elif reading > 0.15:
            status = " ~ edge"

        total_reading += reading

        if i % 5 == 0 or reading > 0.15:
            print(f"    [{i:2d}] lateral={lateral_pos*1000:+5.1f}mm → "
                  f"reading={reading:.3f} (img={img_val:5.1f}){status}")

    print(f"\n📊 Summary:")
    print(f"  Total sensor activation: {total_reading:.3f}")
    print(f"  Sensors on line: {on_sensors}/{QTR_CHANNELS}")
    print(f"  Weighted sum: {(readings**2).sum():.3f}")

    # Compute centroid error
    weights = readings ** 2
    pos_y   = sensors.sensor_pos_body[:, 1]
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
        print(f"    3. Check SPAWN_REGISTRY in config.py for '{TRACK_FILENAME}'")

    # Save a debug image showing robot and sensor positions
    print(f"\n💾 Saving debug visualisation...")
    debug_img  = Image.fromarray(blur_arr.astype(np.uint8)).convert('RGB')
    from PIL import ImageDraw
    draw = ImageDraw.Draw(debug_img)

    rpx, rpy = world_to_pixel(robot.position[0], robot.position[1])
    draw.ellipse([rpx-5, rpy-5, rpx+5, rpy+5], outline=(255, 0, 0), width=2)

    for swx, swy in sensor_xy_world:
        spx2, spy2 = world_to_pixel(swx, swy)
        draw.ellipse([spx2-2, spy2-2, spx2+2, spy2+2], fill=(0, 255, 0))

    out = os.path.join(os.path.dirname(__file__), 'debug_sensor_positions.png')
    debug_img.save(out)
    print(f"  Saved: tests/debug_sensor_positions.png")
    print("="*70)


if __name__ == "__main__":
    main()

