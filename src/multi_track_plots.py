#!/usr/bin/env python3
"""
Visualize multi-track results in a stacked list format.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.gridspec import GridSpec


def plot_multi_track_results(results_dict):
    """
    Plot multi-track test results in a stacked list format.

    Args:
        results_dict: Output from run_multi_track_test()
    """
    track_results = results_dict['track_results']
    num_tracks = len(track_results)

    # Create figure with multiple subplots stacked vertically
    fig = plt.figure(figsize=(14, 2.5 * num_tracks))
    fig.suptitle('Multi-Track Performance Comparison', fontsize=16, fontweight='bold', y=0.995)

    # Color scheme for tracks
    colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6']

    for idx, track_result in enumerate(track_results):
        # Create subplot for each track
        ax = plt.subplot(num_tracks, 1, idx + 1)

        track_name = track_result['track']
        max_error = track_result['max_error'] * 1000  # Convert to mm
        rms_error = track_result['rms_error'] * 1000
        mean_error = track_result['mean_error'] * 1000
        settling_time = track_result['settling_time'] if track_result['settling_time'] else 0
        steady_state = track_result['steady_state_error'] * 1000

        # Determine performance color (red if bad, green if good)
        perf_color = colors[idx % len(colors)]
        if rms_error < 10:
            status_color = '#2ecc71'  # Green - excellent
            status_text = '✓ EXCELLENT'
        elif rms_error < 15:
            status_color = '#3498db'  # Blue - good
            status_text = '✓ GOOD'
        elif rms_error < 25:
            status_color = '#f39c12'  # Orange - ok
            status_text = '⚠ OK'
        else:
            status_color = '#e74c3c'  # Red - poor
            status_text = '✗ POOR'

        # Create table-like layout
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 1)
        ax.axis('off')

        # Background color for the track row
        rect = mpatches.FancyBboxPatch((0, 0), 10, 1,
                                       boxstyle="round,pad=0.02",
                                       linewidth=2,
                                       edgecolor=perf_color,
                                       facecolor='#f8f9fa',
                                       zorder=1)
        ax.add_patch(rect)

        # Track name (left)
        ax.text(0.2, 0.5, f'{track_name.upper()}',
               fontsize=13, fontweight='bold',
               va='center', ha='left', zorder=2)

        # Status indicator (colored box)
        status_box = mpatches.FancyBboxPatch((1.8, 0.2), 1.2, 0.6,
                                            boxstyle="round,pad=0.03",
                                            facecolor=status_color,
                                            edgecolor='none',
                                            zorder=2)
        ax.add_patch(status_box)
        ax.text(2.4, 0.5, status_text,
               fontsize=9, fontweight='bold',
               va='center', ha='center',
               color='white', zorder=3)

        # Performance metrics (right side)
        metrics_text = (
            f"Max: {max_error:.1f}mm  |  "
            f"RMS: {rms_error:.1f}mm  |  "
            f"Mean: {mean_error:.1f}mm  |  "
            f"Settle: {settling_time:.2f}s  |  "
            f"SS: {steady_state:.1f}mm"
        )
        ax.text(9.5, 0.5, metrics_text,
               fontsize=9,
               va='center', ha='right',
               family='monospace',
               zorder=2)

        # Add subtle ranking indicator on far right
        if idx == 0:
            rank_text = '🥇'
        elif idx == 1:
            rank_text = '🥈'
        elif idx == 2:
            rank_text = '🥉'
        else:
            rank_text = f'#{idx+1}'

        # ax.text(9.8, 0.5, rank_text, fontsize=14, va='center', ha='right', zorder=2)

    plt.tight_layout()
    plt.savefig('multi_track_results.png', dpi=150, bbox_inches='tight', facecolor='white')
    print("✅ Saved: multi_track_results.png")
    plt.show()


def plot_multi_track_comparison_bars(results_dict):
    """
    Plot multi-track results as bar charts comparing key metrics.

    Args:
        results_dict: Output from run_multi_track_test()
    """
    track_results = results_dict['track_results']
    track_names = [r['track'].replace('_', ' ').title() for r in track_results]

    max_errors = [r['max_error'] * 1000 for r in track_results]
    rms_errors = [r['rms_error'] * 1000 for r in track_results]
    settling_times = [r['settling_time'] if r['settling_time'] else 0 for r in track_results]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('Multi-Track Performance Metrics', fontsize=14, fontweight='bold')

    colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6']

    # Max Error
    axes[0].bar(track_names, max_errors, color=colors[:len(track_names)], alpha=0.7, edgecolor='black')
    axes[0].set_ylabel('Max Error (mm)', fontweight='bold')
    axes[0].set_title('Maximum Error')
    axes[0].grid(axis='y', alpha=0.3)
    for i, v in enumerate(max_errors):
        axes[0].text(i, v + 1, f'{v:.1f}', ha='center', fontweight='bold')

    # RMS Error
    axes[1].bar(track_names, rms_errors, color=colors[:len(track_names)], alpha=0.7, edgecolor='black')
    axes[1].set_ylabel('RMS Error (mm)', fontweight='bold')
    axes[1].set_title('RMS Error (Tracking Quality)')
    axes[1].grid(axis='y', alpha=0.3)
    for i, v in enumerate(rms_errors):
        axes[1].text(i, v + 0.5, f'{v:.1f}', ha='center', fontweight='bold')

    # Settling Time
    axes[2].bar(track_names, settling_times, color=colors[:len(track_names)], alpha=0.7, edgecolor='black')
    axes[2].set_ylabel('Settling Time (s)', fontweight='bold')
    axes[2].set_title('Settling Time')
    axes[2].grid(axis='y', alpha=0.3)
    for i, v in enumerate(settling_times):
        if v > 0:
            axes[2].text(i, v + 0.05, f'{v:.2f}', ha='center', fontweight='bold')

    # Rotate x labels
    for ax in axes:
        ax.set_xticklabels(track_names, rotation=45, ha='right')

    plt.tight_layout()
    plt.savefig('multi_track_bars.png', dpi=150, bbox_inches='tight', facecolor='white')
    print("✅ Saved: multi_track_bars.png")
    plt.show()


def plot_multi_track_summary_table(results_dict):
    """
    Plot a summary table of all tracks with aggregated stats.

    Args:
        results_dict: Output from run_multi_track_test()
    """
    track_results = results_dict['track_results']

    # Prepare table data
    table_data = []
    for r in track_results:
        table_data.append([
            r['track'].replace('_', ' ').title(),
            f"{r['max_error']*1000:.2f}",
            f"{r['rms_error']*1000:.2f}",
            f"{r['mean_error']*1000:.2f}",
            f"{r['settling_time']:.2f}" if r['settling_time'] else "N/A",
            f"{r['steady_state_error']*1000:.2f}",
        ])

    # Add aggregated row
    table_data.append(['AVERAGE',
                      f"{results_dict['avg_max_error']*1000:.2f}",
                      f"{results_dict['avg_rms_error']*1000:.2f}",
                      f"—",
                      f"{results_dict['avg_settling_time']:.2f}",
                      f"{results_dict['avg_steady_state_error']*1000:.2f}"])

    # Create figure with table
    fig, ax = plt.subplots(figsize=(14, 2 + len(table_data) * 0.5))
    ax.axis('off')

    # Table columns
    columns = ['Track', 'Max (mm)', 'RMS (mm)', 'Mean (mm)', 'Settle (s)', 'SS (mm)']

    # Create table
    table = ax.table(cellText=table_data,
                     colLabels=columns,
                     cellLoc='center',
                     loc='center',
                     colWidths=[0.25, 0.15, 0.15, 0.15, 0.15, 0.15])

    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2.5)

    # Style header
    for i in range(len(columns)):
        table[(0, i)].set_facecolor('#34495e')
        table[(0, i)].set_text_props(weight='bold', color='white')

    # Style data rows with alternating colors
    for i in range(1, len(table_data) + 1):
        for j in range(len(columns)):
            if i == len(table_data):  # Aggregate row
                table[(i, j)].set_facecolor('#3498db')
                table[(i, j)].set_text_props(weight='bold', color='white')
            elif i % 2 == 0:
                table[(i, j)].set_facecolor('#ecf0f1')
            else:
                table[(i, j)].set_facecolor('#ffffff')

    plt.title('Multi-Track Performance Summary', fontsize=14, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.savefig(_out('multi_track_table.png'), dpi=150, bbox_inches='tight', facecolor='white')
    print(f"✅ Saved: output/multi_track_table.png")
    plt.show()


if __name__ == "__main__":
    from multi_track_simulator import run_multi_track_test
    from control.pid_controller import PID, SpeedController

    # Example usage
    pid = PID(kp=90.0, ki=4.0, kd=20.0, limit=20.0)
    speed_controller = SpeedController(straight_speed=0.90, turn_speed=0.65)

    print("Running multi-track test...")
    results = run_multi_track_test(pid, speed_controller)

    if results:
        print("\nGenerating visualizations...")
        plot_multi_track_results(results)
        plot_multi_track_comparison_bars(results)
        plot_multi_track_summary_table(results)
        print("\n✅ All visualizations generated!")

