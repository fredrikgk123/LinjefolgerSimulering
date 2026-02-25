#!/usr/bin/env python3
"""
Quick test script to run multi-track evaluation.
Usage: python3 test_multi_track.py
"""

import sys
sys.path.insert(0, '..')

from control.pid_controller import PID, SpeedController
from multi_track_simulator import run_multi_track_test
from multi_track_plots import (plot_multi_track_results,
                               plot_multi_track_comparison_bars,
                               plot_multi_track_summary_table)
from track.multi_track import get_track_names


def main():
    # Create your current PID and speed controller
    pid = PID(
        kp=90.0, ki=4.0, kd=20.0,
        limit=20.0, integral_limit=1.3, derivative_filter=0.11
    )

    speed_controller = SpeedController(
        straight_speed=0.90,
        turn_speed=0.65,
        error_threshold=0.007,
        smoothing=0.12
    )

    # Show available tracks
    print("\nAvailable tracks:")
    for i, name in enumerate(get_track_names(), 1):
        print(f"  {i}. {name}")

    # Run on all tracks
    print("\nRunning multi-track simulation...")
    results = run_multi_track_test(pid, speed_controller)

    if results:
        print("\n📊 Generating visualizations...")

        # Generate all three types of plots
        plot_multi_track_results(results)
        plot_multi_track_comparison_bars(results)
        plot_multi_track_summary_table(results)

        print("\n💡 Interpretation:")
        avg_rms = results['avg_rms_error'] * 1000
        if avg_rms < 10:
            rating = "✅ EXCELLENT"
        elif avg_rms < 15:
            rating = "✅ GOOD"
        elif avg_rms < 25:
            rating = "⚠️ OK"
        else:
            rating = "❌ POOR"

        print(f"   - Average RMS Error: {avg_rms:.2f}mm")
        print(f"   - Overall Rating: {rating}")
        print(f"   - Tested across {results['num_tracks']} different tracks")


if __name__ == "__main__":
    main()

