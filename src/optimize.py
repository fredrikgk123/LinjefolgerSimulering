#!/usr/bin/env python3
"""
Quick start script for PID optimization.
Run this to automatically find optimal PID parameters.
"""

import sys
import argparse
from pid_optimizer import PIDOptimizer
import numpy as np


def run_quick_optimization():
    """Run a quick optimization with default parameters."""
    print("\n" + "="*70)
    print("QUICK PID OPTIMIZATION")
    print("="*70)

    # Coarse search space (faster)
    param_ranges = {
        'kp': np.linspace(70, 120, 3),
        'ki': np.linspace(3, 5, 3),
        'kd': np.linspace(18, 24, 3),
        'limit': np.linspace(18, 22, 2),
        'straight_speed': np.linspace(0.85, 0.95, 2),
        'turn_speed': np.linspace(0.65, 0.75, 2),
    }

    optimizer = PIDOptimizer(param_ranges=param_ranges)
    optimizer.grid_search()
    optimizer.save_results('optimization_results_quick.json')
    optimizer.plot_optimization_results()

    return optimizer


def run_full_optimization():
    """Run a full optimization with fine search space."""
    print("\n" + "="*70)
    print("FULL PID OPTIMIZATION")
    print("="*70)

    # Fine search space (slower, more accurate)
    param_ranges = {
        'kp': np.linspace(60, 130, 8),
        'ki': np.linspace(2.5, 5.5, 7),
        'kd': np.linspace(15, 25, 6),
        'limit': np.linspace(16, 24, 5),
        'straight_speed': np.linspace(0.80, 1.0, 4),
        'turn_speed': np.linspace(0.55, 0.80, 4),
    }

    optimizer = PIDOptimizer(param_ranges=param_ranges)
    optimizer.grid_search()
    optimizer.save_results('optimization_results_full.json')
    optimizer.plot_optimization_results()

    return optimizer


def run_bayesian_optimization(n_iterations=30):
    """Run bayesian optimization (random sampling with best tracking)."""
    print("\n" + "="*70)
    print("BAYESIAN PID OPTIMIZATION")
    print("="*70)

    param_ranges = {
        'kp': np.linspace(60, 130, 15),
        'ki': np.linspace(2.5, 5.5, 15),
        'kd': np.linspace(15, 25, 15),
        'limit': np.linspace(16, 24, 10),
        'straight_speed': np.linspace(0.80, 1.5, 10),
        'turn_speed': np.linspace(0.55, 1.00, 10),
    }

    optimizer = PIDOptimizer(param_ranges=param_ranges, optimization_method='bayesian')
    optimizer.bayesian_search(n_iterations=n_iterations)
    optimizer.save_results('optimization_results_bayesian.json')
    optimizer.plot_optimization_results()

    return optimizer


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='PID Parameter Optimizer')
    parser.add_argument('--mode', '-mode', choices=['quick', 'full', 'bayesian'],
                       default='quick',
                       help='Optimization mode (default: quick)')
    parser.add_argument('--iterations', '-iterations', type=int, default=30,
                       help='Number of iterations for Bayesian (default: 30)')

    args = parser.parse_args()

    if args.mode == 'quick':
        print("⚡ Quick mode: ~27 combinations (~13 minutes)")
        optimizer = run_quick_optimization()
    elif args.mode == 'full':
        print("🔬 Full mode: ~6,400 combinations (~50+ hours)")
        response = input("This will take a very long time. Continue? (y/n): ")
        if response.lower() == 'y':
            optimizer = run_full_optimization()
        else:
            print("Cancelled.")
            sys.exit(0)
    elif args.mode == 'bayesian':
        print(f"🎲 Bayesian mode: {args.iterations} random samples (~{args.iterations*0.5:.0f} minutes)")
        optimizer = run_bayesian_optimization(n_iterations=args.iterations)

    print("\n✨ Optimization complete!")

