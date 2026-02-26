#!/usr/bin/env python3
"""
preview_track.py

Plots a track image with the start/finish zone, spawn point, departure
threshold, and checkpoints marked — lets you visually verify lap-optimizer
settings before running it.

Usage:
    cd src
    python3 preview_track.py                              # bane_fase2 (default)
    python3 preview_track.py --track ../assets/suzuka.png
"""

import os
import argparse
import numpy as np
import matplotlib
try:
    matplotlib.use("TkAgg")
except Exception:
    pass
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import ImageFilter

from track.image_loader import load_track_image
from config import (
    MAP_SIZE_M,
    SPAWN_REGISTRY,
    START_FINISH_RADIUS,
    MIN_DEPARTURE_DIST,
    MAX_LATERAL_ERROR,
    MAX_LAP_TIME,
    CHECKPOINT_REGISTRY,
    CHECKPOINT_RADIUS,
)

_HERE       = os.path.dirname(__file__)
_ASSETS_DIR = os.path.normpath(os.path.join(_HERE, '..', 'assets'))
_OUTPUT_DIR = os.path.normpath(os.path.join(_HERE, '..', 'output'))
os.makedirs(_OUTPUT_DIR, exist_ok=True)


def preview(track_filename: str = "bane_fase2.png"):
    W, H = MAP_SIZE_M

    track_path = os.path.join(_ASSETS_DIR, track_filename)
    track      = load_track_image(track_path)
    blurred    = track.filter(ImageFilter.GaussianBlur(radius=2))
    track_arr  = np.array(blurred)

    if track_filename not in SPAWN_REGISTRY:
        print(f"[warn] No spawn entry for '{track_filename}' in config.py — using centre.")
        spawn = {"x": W / 2, "y": H / 2, "theta": 0.0}
    else:
        spawn = SPAWN_REGISTRY[track_filename]

    sx, sy = spawn["x"], spawn["y"]

    fig, ax = plt.subplots(figsize=(12, 7))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')

    ax.imshow(track_arr, cmap='gray', origin='upper',
              extent=[0, W, 0, H], aspect='equal')

    # ── Start/finish zone ─────────────────────────────────────────────────────
    ax.add_patch(plt.Circle((sx, sy), START_FINISH_RADIUS,
                            color='#00ff88', alpha=0.25, zorder=3))
    ax.add_patch(plt.Circle((sx, sy), START_FINISH_RADIUS,
                            fill=False, edgecolor='#00ff88',
                            linewidth=2.5, linestyle='--', zorder=4))

    # ── Min departure circle ──────────────────────────────────────────────────
    ax.add_patch(plt.Circle((sx, sy), MIN_DEPARTURE_DIST,
                            fill=False, edgecolor='#ffaa00',
                            linewidth=1.5, linestyle=':', zorder=4))

    # ── Spawn point + heading arrow ───────────────────────────────────────────
    ax.plot(sx, sy, 'o', color='#00ff88', markersize=10, zorder=5)
    arrow_len = 0.12
    ax.annotate("",
                xy=(sx + arrow_len * np.cos(spawn["theta"]),
                    sy + arrow_len * np.sin(spawn["theta"])),
                xytext=(sx, sy),
                arrowprops=dict(arrowstyle="->", color='#00ff88', lw=2.5))

    # ── Checkpoints ───────────────────────────────────────────────────────────
    checkpoints = CHECKPOINT_REGISTRY.get(track_filename, [])
    for i, (cx, cy) in enumerate(checkpoints):
        ax.add_patch(plt.Circle((cx, cy), CHECKPOINT_RADIUS,
                                color='#ffd700', alpha=0.20, zorder=2))
        ax.add_patch(plt.Circle((cx, cy), CHECKPOINT_RADIUS,
                                fill=False, edgecolor='#ffd700',
                                linewidth=2.0, zorder=4))
        ax.text(cx, cy, str(i + 1), color='#ffd700',
                fontsize=10, fontweight='bold',
                ha='center', va='center', zorder=5)

    # ── Labels ────────────────────────────────────────────────────────────────
    ax.text(sx + START_FINISH_RADIUS + 0.03, sy,
            f"Start/Finish\nr = {START_FINISH_RADIUS*100:.0f} cm",
            color='#00ff88', fontsize=9, va='center')
    ax.text(sx + MIN_DEPARTURE_DIST + 0.03, sy - 0.08,
            f"Min departure\nr = {MIN_DEPARTURE_DIST*100:.0f} cm",
            color='#ffaa00', fontsize=8, va='center')

    # ── Info box ──────────────────────────────────────────────────────────────
    info = (f"Spawn:  ({sx:.2f}, {sy:.2f})  θ={np.degrees(spawn['theta']):.0f}°\n"
            f"Max lateral error:  {MAX_LATERAL_ERROR*1000:.0f} mm\n"
            f"Max lap time:       {MAX_LAP_TIME:.0f} s\n"
            f"Checkpoints:        {len(checkpoints)}")
    ax.text(0.02, 0.98, info, transform=ax.transAxes,
            fontsize=9, color='white', va='top', ha='left',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#2a2a4a',
                      edgecolor='#555577', alpha=0.9))

    # ── Legend ────────────────────────────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor='#00ff88', alpha=0.4, label='Start/Finish zone'),
        mpatches.Patch(facecolor='none', edgecolor='#ffaa00',
                       linestyle=':', label='Min departure radius'),
        mpatches.Patch(facecolor='#ffd700', alpha=0.4, label='Checkpoint zone'),
    ]
    ax.legend(handles=legend_handles, loc='lower right',
              facecolor='#2a2a4a', edgecolor='#555577',
              labelcolor='white', fontsize=9)

    # ── Axes ──────────────────────────────────────────────────────────────────
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_xlabel('x (m)', color='white')
    ax.set_ylabel('y (m)', color='white')
    ax.tick_params(colors='white')
    for spine in ax.spines.values():
        spine.set_edgecolor('#555577')
    ax.set_title(f'{track_filename}  —  Track Preview',
                 color='white', fontsize=13, fontweight='bold', pad=12)

    plt.tight_layout()
    out_path = os.path.join(_OUTPUT_DIR, 'track_preview.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    print(f"Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Preview track with spawn + checkpoints")
    parser.add_argument("--track", default="bane_fase2.png",
                        help="Track filename in assets/ (default: bane_fase2.png)")
    args = parser.parse_args()
    preview(args.track)
