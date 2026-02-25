#!/usr/bin/env python3
"""
Performance analysis utility for the line-following robot simulation.
Calculates key metrics from simulation data.
"""

import numpy as np

def analyze_tracking_performance(err_log, dt=0.005):
    """
    Analyze tracking performance from error log.

    Args:
        err_log: List or array of lateral errors over time
        dt: Time step in seconds

    Returns:
        dict with performance metrics
    """
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


def compare_pid_configs(configs_and_results):
    """
    Compare multiple PID configurations.

    Args:
        configs_and_results: List of tuples (config_name, err_log)
    """
    print("\n" + "="*70)
    print("  PID CONFIGURATION COMPARISON")
    print("="*70)
    print(f"\n{'Config':<20} {'Max(mm)':<10} {'RMS(mm)':<10} {'Settle(s)':<12} {'SS(mm)':<10}")
    print("-"*70)

    for name, err_log in configs_and_results:
        metrics = analyze_tracking_performance(err_log)
        settle_str = f"{metrics['settling_time']:.2f}" if metrics['settling_time'] else "N/A"
        print(f"{name:<20} {metrics['max_error']*1000:<10.2f} {metrics['rms_error']*1000:<10.2f} "
              f"{settle_str:<12} {metrics['steady_state_error']*1000:<10.2f}")

    print("="*70 + "\n")


# Example usage (can be integrated into main.py):
if __name__ == "__main__":
    # Simulate some test data
    print("Performance Analysis Utility")
    print("----------------------------")
    print("To use this in your simulation, import and call:")
    print("  from performance_metrics import analyze_tracking_performance, print_performance_report")
    print("\nThen at the end of your simulation:")
    print("  metrics = analyze_tracking_performance(err_log, DT)")
    print("  print_performance_report(metrics)")

    # Example with synthetic data
    print("\nExample with synthetic error data:")
    t = np.linspace(0, 15, 3000)
    err_example = 0.04 * np.exp(-t/2) * np.sin(10*t)  # Damped oscillation

    metrics = analyze_tracking_performance(err_example, dt=0.005)
    print_performance_report(metrics)

