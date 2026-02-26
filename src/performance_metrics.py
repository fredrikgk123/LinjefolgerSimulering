#!/usr/bin/env python3
"""
Performance analysis utility for the line-following robot simulation.
Calculates key metrics from simulation data.
"""

import numpy as np
from config import DT


def analyze_tracking_performance(err_log, dt=None):
    """
    Analyze tracking performance from error log.

    Args:
        err_log: List or array of lateral errors over time
        dt: Time step in seconds (defaults to DT from config.py)

    Returns:
        dict with performance metrics
    """
    if dt is None:
        dt = DT

    errors = np.array(err_log)
    time = np.arange(len(errors)) * dt

    # Basic statistics
    max_error = np.max(np.abs(errors))
    mean_error = np.mean(np.abs(errors))
    rms_error = np.sqrt(np.mean(errors**2))

    # Settling time (time to reach < 0.01m error)
    settling_threshold = 0.01
    settled_mask = np.abs(errors) < settling_threshold
    if np.any(settled_mask):
        settling_idx = np.where(settled_mask)[0][0]
        settling_time = settling_idx * dt
    else:
        settling_time = None

    # Steady-state error (last 20% of trajectory)
    steady_start = int(0.8 * len(errors))
    steady_state_error = np.mean(np.abs(errors[steady_start:]))

    # Oscillation detection (zero crossings in steady state)
    if len(errors[steady_start:]) > 10:
        zero_crossings = np.sum(np.diff(np.sign(errors[steady_start:])) != 0)
        oscillation_freq = zero_crossings / (len(errors[steady_start:]) * dt)
    else:
        oscillation_freq = 0

    return {
        'max_error': max_error,
        'mean_error': mean_error,
        'rms_error': rms_error,
        'settling_time': settling_time,
        'steady_state_error': steady_state_error,
        'oscillation_frequency': oscillation_freq,
        'total_time': time[-1] if len(time) > 0 else 0
    }


def print_performance_report(metrics):
    """Print formatted performance report."""
    print("\n" + "="*50)
    print("  LINE-FOLLOWING PERFORMANCE REPORT")
    print("="*50)
    print(f"\n📊 Error Statistics:")
    print(f"  • Maximum Error:      {metrics['max_error']*1000:.2f} mm")
    print(f"  • Mean Absolute Error: {metrics['mean_error']*1000:.2f} mm")
    print(f"  • RMS Error:          {metrics['rms_error']*1000:.2f} mm")
    print(f"\n⏱️  Temporal Metrics:")
    if metrics['settling_time'] is not None:
        print(f"  • Settling Time:      {metrics['settling_time']:.3f} s")
    else:
        print(f"  • Settling Time:      Never settled")
    print(f"  • Total Sim Time:     {metrics['total_time']:.2f} s")
    print(f"\n📈 Steady-State Performance:")
    print(f"  • Steady-State Error: {metrics['steady_state_error']*1000:.2f} mm")
    print(f"  • Oscillation Freq:   {metrics['oscillation_frequency']:.2f} Hz")
    print("="*50 + "\n")

