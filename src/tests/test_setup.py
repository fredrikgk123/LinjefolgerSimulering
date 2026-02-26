#!/usr/bin/env python3
"""
Quick test script to verify the simulation setup is correct.
Runs basic checks without opening GUI windows.
"""

import sys
import os

# ── add src/ to path ──────────────────────────────────────────────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

_HERE       = os.path.dirname(__file__)
_ASSETS_DIR = os.path.normpath(os.path.join(_HERE, '..', '..', 'assets'))


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
        ('config',                  'config'),
        ('track.image_loader',      'track image loader'),
        ('sensors.qtr_array',       'sensors'),
        ('physics.robot_model',     'robot model'),
        ('physics.friction',        'friction'),
        ('control.pid_controller',  'PID controller'),
        ('performance_metrics',     'performance metrics'),
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
        import numpy as np
        from PIL import ImageFilter
        from track.image_loader import load_track_image
        from sensors.qtr_array import QTRArray
        from physics.robot_model import Robot
        from control.pid_controller import PID
        import config

        # Load real track image
        track_path = os.path.join(_ASSETS_DIR, "bane_fase2.png")
        if not os.path.exists(track_path):
            # Fall back to suzuka if bane_fase2 is missing
            track_path = os.path.join(_ASSETS_DIR, "suzuka.png")
        track    = load_track_image(track_path)
        blurred  = track.filter(ImageFilter.GaussianBlur(radius=2))
        blur_arr = np.array(blurred, dtype=np.float32)
        print(f"  ✓ Track loaded  ({os.path.basename(track_path)})")

        # Create robot using the spawn registry
        robot = Robot()
        track_name = os.path.basename(track_path)
        if track_name in config.SPAWN_REGISTRY:
            sp = config.SPAWN_REGISTRY[track_name]
            robot.position = np.array([sp["x"], sp["y"]])
            robot.theta    = sp["theta"]
        else:
            W, H = config.MAP_SIZE_M
            robot.position = np.array([W / 2, H / 2])
        print("  ✓ Robot initialised")

        # Create sensors
        sensors = QTRArray()
        print("  ✓ Sensor array")

        # Create PID
        pid = PID(kp=config.PID_KP, ki=config.PID_KI, kd=config.PID_KD,
                  limit=config.PID_LIMIT)
        print("  ✓ PID controller")

        # Test one simulation step
        readings = sensors.read(robot, blur_arr)
        weights  = readings ** 2
        pos_y    = sensors.sensor_pos_body[:, 1]
        total_w  = weights.sum()

        if total_w > 0.08:
            e_y = float(np.dot(weights, pos_y) / total_w)
        else:
            e_y = 0.0

        w_cmd  = pid.compute(e_y, config.DT)
        v_cmd  = 0.40
        vL_cmd = v_cmd - w_cmd * config.WHEEL_BASE / 2
        vR_cmd = v_cmd + w_cmd * config.WHEEL_BASE / 2
        robot.update(vL_cmd, vR_cmd)
        print("  ✓ Simulation step")

        # Test performance metrics
        from performance_metrics import analyze_tracking_performance
        test_errors = [0.01, 0.02, 0.01, 0.005, 0.003]
        analyze_tracking_performance(test_errors)
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

    results = [
        ("Dependencies",    test_imports()),
        ("Project Modules", test_project_modules()),
        ("Simulation",      test_basic_simulation()),
    ]

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

