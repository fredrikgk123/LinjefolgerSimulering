#!/usr/bin/env python3
"""
Quick test script to verify the simulation setup is correct.
Runs basic checks without opening GUI windows.
"""

import sys
import os

def test_imports():
    """Test that all required modules can be imported."""
    print("Testing imports...")
    try:
        import numpy as np
        print("  ✓ numpy")
    except ImportError as e:
        print(f"  ✗ numpy: {e}")
        return False

    try:
        from PIL import Image, ImageFilter, ImageDraw
        print("  ✓ Pillow")
    except ImportError as e:
        print(f"  ✗ Pillow: {e}")
        return False

    try:
        import matplotlib
        import matplotlib.pyplot as plt
        print("  ✓ matplotlib")
    except ImportError as e:
        print(f"  ✗ matplotlib: {e}")
        return False

    return True

def test_project_modules():
    """Test that project modules can be imported."""
    print("\nTesting project modules...")

    modules = [
        ('config', 'config'),
        ('track.track_generator', 'track generator'),
        ('sensors.qtr_array', 'sensors'),
        ('physics.robot_model', 'robot model'),
        ('physics.friction', 'friction'),
        ('control.pid_controller', 'PID controller'),
        ('performance_metrics', 'performance metrics'),
    ]

    all_ok = True
    for module_name, display_name in modules:
        try:
            __import__(module_name)
            print(f"  ✓ {display_name}")
        except ImportError as e:
            print(f"  ✗ {display_name}: {e}")
            all_ok = False

    return all_ok

def test_basic_simulation():
    """Test basic simulation components without GUI."""
    print("\nTesting simulation components...")

    try:
        from track.track_generator import generate_default_track
        from sensors.qtr_array import QTRArray
        from physics.robot_model import Robot
        from control.pid_controller import PID
        import config
        import numpy as np
        from PIL import ImageFilter

        # Generate track
        track = generate_default_track()
        blurred = track.filter(ImageFilter.GaussianBlur(radius=2))
        blur_arr = np.array(blurred, dtype=np.float32)
        print("  ✓ Track generation")

        # Create robot
        robot = Robot()
        W, H = config.MAP_SIZE_M
        robot.position = np.array([0.25, 1.5])
        robot.theta = 0.0
        print("  ✓ Robot initialization")

        # Create sensors
        sensors = QTRArray()
        print("  ✓ Sensor array")

        # Create PID
        pid = PID(kp=6.5, ki=0.3, kd=0.25)
        print("  ✓ PID controller")

        # Test one simulation step
        readings = sensors.read(robot, blur_arr)
        weights = readings ** 2
        pos_y = sensors.sensor_pos_body[:, 1]
        total_w = weights.sum()

        if total_w > 0.08:
            e_y = float(np.dot(weights, pos_y) / total_w)
        else:
            e_y = 0.0

        w_cmd = pid.compute(e_y, config.DT)
        v_cmd = 0.40

        vL_cmd = v_cmd - w_cmd * config.WHEEL_BASE / 2
        vR_cmd = v_cmd + w_cmd * config.WHEEL_BASE / 2

        robot.update(vL_cmd, vR_cmd)
        print("  ✓ Simulation step")

        # Test performance metrics
        from performance_metrics import analyze_tracking_performance
        test_errors = [0.01, 0.02, 0.01, 0.005, 0.003]
        metrics = analyze_tracking_performance(test_errors)
        print("  ✓ Performance metrics")

        return True

    except Exception as e:
        print(f"  ✗ Simulation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("="*60)
    print("Line-Following Robot Simulation - System Check")
    print("="*60)

    results = []

    # Run tests
    results.append(("Dependencies", test_imports()))
    results.append(("Project Modules", test_project_modules()))
    results.append(("Simulation", test_basic_simulation()))

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{name:.<40} {status}")
        all_passed = all_passed and passed

    print("="*60)

    if all_passed:
        print("\n✨ All checks passed! You're ready to run the simulation.")
        print("\nTo start the simulation, run:")
        print("  python3 main.py")
        return 0
    else:
        print("\n⚠️  Some checks failed. Please review the errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())

