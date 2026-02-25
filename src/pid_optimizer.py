#!/usr/bin/env python3
"""
Machine Learning loop to optimize PID controller parameters.
Tests multiple parameter combinations and finds the best tuning.
"""

import numpy as np
import json
from datetime import datetime
from control.pid_controller import PID, SpeedController
from multi_track_simulator import run_multi_track_test
from multi_track_plots import plot_multi_track_results, plot_multi_track_comparison_bars, plot_multi_track_summary_table


class PIDOptimizer:
    """Optimize PID parameters using grid search or bayesian optimization."""

    def __init__(self, param_ranges=None, optimization_method='grid'):
        """
        Initialize the optimizer.

        Args:
            param_ranges: dict with parameter ranges
            optimization_method: 'grid' for grid search, 'bayesian' for bayesian optimization
        """
        self.optimization_method = optimization_method

        # Default parameter ranges to search
        if param_ranges is None:
            self.param_ranges = {
                'kp': np.linspace(50, 150, 5),      # Kp: 50-150
                'ki': np.linspace(2, 6, 5),         # Ki: 2-6
                'kd': np.linspace(12, 28, 5),       # Kd: 12-28
                'limit': np.linspace(15, 25, 3),    # limit: 15-25
                'straight_speed': np.linspace(0.80, 1.0, 3),  # 0.80-1.0
                'turn_speed': np.linspace(0.50, 0.75, 3),     # 0.50-0.75
            }
        else:
            self.param_ranges = param_ranges

        self.results = []
        self.best_result = None
        self.best_params = None
        self.iteration = 0
        self.start_time = None

    def evaluate_params(self, kp, ki, kd, limit, straight_speed, turn_speed):
        """
        Test a specific parameter combination on all tracks.

        Returns:
            avg_rms_error: Average RMS error across all tracks (lower is better)
        """
        pid = PID(
            kp=kp, ki=ki, kd=kd,
            limit=limit, integral_limit=1.3, derivative_filter=0.11
        )

        speed_controller = SpeedController(
            straight_speed=straight_speed,
            turn_speed=turn_speed,
            error_threshold=0.007,
            smoothing=0.12
        )

        try:
            results = run_multi_track_test(pid, speed_controller, show_viz=False)
            avg_rms_error = results['avg_rms_error']
            return results, avg_rms_error
        except Exception as e:
            print(f"ERROR evaluating params: {e}")
            return None, float('inf')

    def grid_search(self, n_best=3):
        """
        Perform exhaustive grid search over parameter space.

        Args:
            n_best: Number of best results to keep and refine
        """
        print("\n" + "="*70)
        print("STARTING GRID SEARCH OPTIMIZATION")
        print("="*70)

        self.start_time = datetime.now()

        # Calculate total combinations
        total_combos = 1
        for key in self.param_ranges:
            total_combos *= len(self.param_ranges[key])

        print(f"Total combinations to test: {total_combos}")
        print(f"Estimated time: ~{total_combos * 0.5:.0f} minutes")
        print("="*70 + "\n")

        # Generate all combinations
        keys = list(self.param_ranges.keys())
        ranges = [self.param_ranges[k] for k in keys]

        # Iterate through all combinations
        for combo in np.ndindex(tuple(len(r) for r in ranges)):
            self.iteration += 1

            # Build parameter dict for this iteration
            params = {keys[i]: ranges[i][combo[i]] for i in range(len(keys))}

            # Evaluate
            print(f"[{self.iteration}] Testing: Kp={params['kp']:.1f}, Ki={params['ki']:.1f}, "
                  f"Kd={params['kd']:.1f}, Speed={params['straight_speed']:.2f}")

            results, avg_rms = self.evaluate_params(**params)

            if results is not None:
                result_entry = {
                    'iteration': self.iteration,
                    'params': params,
                    'avg_rms_error': avg_rms,
                    'avg_max_error': results['avg_max_error'],
                    'avg_settling_time': results['avg_settling_time'],
                    'track_results': results['track_results']
                }
                self.results.append(result_entry)

                # Update best
                if self.best_result is None or avg_rms < self.best_result['avg_rms_error']:
                    self.best_result = result_entry
                    self.best_params = params
                    print(f"   ✨ NEW BEST! RMS Error: {avg_rms*1000:.2f}mm\n")
                else:
                    print(f"   RMS Error: {avg_rms*1000:.2f}mm\n")

        self._print_summary()

    def bayesian_search(self, n_iterations=20):
        """
        Perform Bayesian optimization (simplified grid refinement approach).

        Args:
            n_iterations: Number of iterations to run
        """
        print("\n" + "="*70)
        print("STARTING BAYESIAN OPTIMIZATION")
        print("="*70)
        print(f"Iterations: {n_iterations}")
        print("="*70 + "\n")

        self.start_time = datetime.now()

        # Start with coarse grid, refine based on results
        for iteration in range(n_iterations):
            self.iteration = iteration + 1

            # Random sampling from parameter space
            params = {
                'kp': np.random.choice(self.param_ranges['kp']),
                'ki': np.random.choice(self.param_ranges['ki']),
                'kd': np.random.choice(self.param_ranges['kd']),
                'limit': np.random.choice(self.param_ranges['limit']),
                'straight_speed': np.random.choice(self.param_ranges['straight_speed']),
                'turn_speed': np.random.choice(self.param_ranges['turn_speed']),
            }

            print(f"[{self.iteration}/{n_iterations}] Testing: Kp={params['kp']:.1f}, "
                  f"Ki={params['ki']:.1f}, Kd={params['kd']:.1f}")

            results, avg_rms = self.evaluate_params(**params)

            if results is not None:
                result_entry = {
                    'iteration': self.iteration,
                    'params': params,
                    'avg_rms_error': avg_rms,
                    'avg_max_error': results['avg_max_error'],
                    'avg_settling_time': results['avg_settling_time'],
                }
                self.results.append(result_entry)

                if self.best_result is None or avg_rms < self.best_result['avg_rms_error']:
                    self.best_result = result_entry
                    self.best_params = params
                    print(f"   ✨ NEW BEST! RMS Error: {avg_rms*1000:.2f}mm\n")
                else:
                    print(f"   RMS Error: {avg_rms*1000:.2f}mm\n")

        self._print_summary()

    def _print_summary(self):
        """Print optimization summary."""
        elapsed = datetime.now() - self.start_time

        print("\n" + "="*70)
        print("OPTIMIZATION COMPLETE")
        print("="*70)
        print(f"Total iterations: {self.iteration}")
        print(f"Time elapsed: {elapsed}")
        print(f"\n🏆 BEST PARAMETERS FOUND:")
        print(f"  Kp: {self.best_params['kp']:.2f}")
        print(f"  Ki: {self.best_params['ki']:.2f}")
        print(f"  Kd: {self.best_params['kd']:.2f}")
        print(f"  Limit: {self.best_params['limit']:.2f}")
        print(f"  Straight Speed: {self.best_params['straight_speed']:.3f}")
        print(f"  Turn Speed: {self.best_params['turn_speed']:.3f}")
        print(f"\n📊 BEST PERFORMANCE:")
        print(f"  Average RMS Error: {self.best_result['avg_rms_error']*1000:.2f}mm")
        print(f"  Average Max Error: {self.best_result['avg_max_error']*1000:.2f}mm")
        print(f"  Average Settling Time: {self.best_result['avg_settling_time']:.2f}s")
        print("="*70)

    def get_top_n(self, n=5):
        """Get top N best results."""
        sorted_results = sorted(self.results, key=lambda x: x['avg_rms_error'])
        return sorted_results[:n]

    def save_results(self, filename='optimization_results.json'):
        """Save optimization results to file."""
        data = {
            'timestamp': datetime.now().isoformat(),
            'total_iterations': self.iteration,
            'best_params': self.best_params,
            'best_rms_error': float(self.best_result['avg_rms_error']),
            'best_max_error': float(self.best_result['avg_max_error']),
            'all_results': [
                {
                    'iteration': r['iteration'],
                    'params': {k: float(v) for k, v in r['params'].items()},
                    'avg_rms_error': float(r['avg_rms_error']),
                    'avg_max_error': float(r['avg_max_error']),
                }
                for r in self.results
            ]
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"\n✅ Results saved to: {filename}")

    def plot_optimization_results(self):
        """Generate visualizations for best result."""
        if self.best_result is None:
            print("No results to plot!")
            return

        print("\n📊 Generating visualizations for best result...")

        # Create a results dict in the format expected by plotting functions
        results_for_plot = {
            'num_tracks': len(self.best_result['track_results']),
            'avg_max_error': self.best_result['avg_max_error'],
            'avg_rms_error': self.best_result['avg_rms_error'],
            'avg_settling_time': self.best_result['avg_settling_time'],
            'track_results': self.best_result['track_results']
        }

        try:
            plot_multi_track_results(results_for_plot)
            plot_multi_track_comparison_bars(results_for_plot)
            print("✅ Visualizations generated!")
        except Exception as e:
            print(f"Could not generate visualizations: {e}")


def main():
    """Run optimization."""

    # Define parameter search space
    param_ranges = {
        'kp': np.linspace(60, 130, 4),        # Focus on this range
        'ki': np.linspace(2.5, 5.5, 4),       # Focus on this range
        'kd': np.linspace(15, 25, 4),         # Focus on this range
        'limit': np.linspace(16, 22, 3),      # Limit for turning
        'straight_speed': np.linspace(0.85, 0.95, 3),
        'turn_speed': np.linspace(0.60, 0.75, 3),
    }

    # Choose optimization method
    # For first run, use grid search (exhaustive, finds global optimum)
    # For refinement, use bayesian (faster, good for fine-tuning)

    optimizer = PIDOptimizer(param_ranges=param_ranges, optimization_method='grid')

    print("\n🤖 PID PARAMETER OPTIMIZATION")
    print("Using: Grid Search (exhaustive)")
    print(f"Search space size: ~{4*4*4*3*3*3:.0f} combinations")

    # Run grid search
    optimizer.grid_search()

    # Get top 5 results
    top_5 = optimizer.get_top_n(5)
    print("\n🏆 TOP 5 BEST RESULTS:")
    for i, result in enumerate(top_5, 1):
        print(f"\n{i}. RMS Error: {result['avg_rms_error']*1000:.2f}mm")
        print(f"   Kp={result['params']['kp']:.1f}, Ki={result['params']['ki']:.1f}, "
              f"Kd={result['params']['kd']:.1f}")
        print(f"   Speed: {result['params']['straight_speed']:.2f} / {result['params']['turn_speed']:.2f}")

    # Save results
    optimizer.save_results('optimization_results.json')

    # Generate visualizations
    optimizer.plot_optimization_results()

    # Print best parameters in code format
    print("\n" + "="*70)
    print("USE THESE PARAMETERS IN YOUR CODE:")
    print("="*70)
    print(f"""
pid = PID(
    kp={optimizer.best_params['kp']:.1f}, 
    ki={optimizer.best_params['ki']:.1f}, 
    kd={optimizer.best_params['kd']:.1f},
    limit={optimizer.best_params['limit']:.1f}, 
    integral_limit=1.3, 
    derivative_filter=0.11
)

speed_controller = SpeedController(
    straight_speed={optimizer.best_params['straight_speed']:.3f},
    turn_speed={optimizer.best_params['turn_speed']:.3f},
    error_threshold=0.007,
    smoothing=0.12
)
""")
    print("="*70)


if __name__ == "__main__":
    main()

