#!/usr/bin/env python3
"""performance_metrics.py — tracking quality statistics from a simulation run."""

import numpy as np
from config import DT


def analyze_tracking_performance(err_log, dt=None):
    if dt is None:
        dt = DT
    errors = np.array(err_log)
    time   = np.arange(len(errors)) * dt

    max_error  = np.max(np.abs(errors))
    mean_error = np.mean(np.abs(errors))
    rms_error  = np.sqrt(np.mean(errors**2))

    settled_mask = np.abs(errors) < 0.01
    settling_time = (np.where(settled_mask)[0][0] * dt
                     if np.any(settled_mask) else None)

    steady_start      = int(0.8 * len(errors))
    steady_state_error = np.mean(np.abs(errors[steady_start:]))

    if len(errors[steady_start:]) > 10:
        zero_crossings  = np.sum(np.diff(np.sign(errors[steady_start:])) != 0)
        oscillation_freq = zero_crossings / (len(errors[steady_start:]) * dt)
    else:
        oscillation_freq = 0

    return {
        'max_error':           max_error,
        'mean_error':          mean_error,
        'rms_error':           rms_error,
        'settling_time':       settling_time,
        'steady_state_error':  steady_state_error,
        'oscillation_frequency': oscillation_freq,
        'total_time':          time[-1] if len(time) > 0 else 0,
    }


def print_performance_report(metrics):
    print("\n" + "="*50)
    print("  LINE-FOLLOWING PERFORMANCE REPORT")
    print("="*50)
    print(f"  Max error:        {metrics['max_error']*1000:.2f} mm")
    print(f"  Mean abs error:   {metrics['mean_error']*1000:.2f} mm")
    print(f"  RMS error:        {metrics['rms_error']*1000:.2f} mm")
    if metrics['settling_time'] is not None:
        print(f"  Settling time:    {metrics['settling_time']:.3f} s")
    else:
        print(f"  Settling time:    never settled")
    print(f"  Total sim time:   {metrics['total_time']:.2f} s")
    print(f"  Steady-state err: {metrics['steady_state_error']*1000:.2f} mm")
    print(f"  Oscillation freq: {metrics['oscillation_frequency']:.2f} Hz")
    print("="*50 + "\n")
