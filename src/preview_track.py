#!/usr/bin/env python3
"""
preview_track.py

Plots bane_fase2 with the start/finish zone and spawn point marked,
so you can visually verify lap optimizer settings before running it.

Usage:
    cd src
    python3 preview_track.py
"""

import os
import numpy as np
from datetime import datetime
import matplotlib
try:
    matplotlib.use("TkAgg")
except Exception:
    pass
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import ImageFilter

# ── resolve paths ──────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(__file__)
_ASSETS_DIR = os.path.normpath(os.path.join(_HERE, '..', 'assets'))
_OUTPUT_DIR = os.path.normpath(os.path.join(_HERE, '..', 'output'))
os.makedirs(_OUTPUT_DIR, exist_ok=True)

from track.image_loader import load_track_image
from lap_optimizer import (SPAWN, START_FINISH_RADIUS,
                           MAX_LATERAL_ERROR, MIN_DEPARTURE_DIST, MAX_LAP_TIME)
from config import MAP_SIZE_M


def preview():
    W, H = MAP_SIZE_M

    # Load track
    track_path = os.path.join(_ASSETS_DIR, 'bane_fase2.png')
    track      = load_track_image(track_path)
    blurred    = track.filter(ImageFilter.GaussianBlur(radius=2))
    track_arr  = np.array(blurred)

    fig, ax = plt.subplots(figsize=(12, 7))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')

    # ── Track image ────────────────────────────────────────────────────────────
    ax.imshow(track_arr, cmap='gray', origin='upper',
              extent=[0, W, 0, H], aspect='equal')

    sx, sy = SPAWN["x"], SPAWN["y"]

    # ── Start/finish zone ──────────────────────────────────────────────────────
    sf_circle = plt.Circle((sx, sy), START_FINISH_RADIUS,
                            color='#00ff88', alpha=0.25, zorder=3)
    sf_border = plt.Circle((sx, sy), START_FINISH_RADIUS,
                            fill=False, edgecolor='#00ff88',
                            linewidth=2.5, linestyle='--', zorder=4)
    ax.add_patch(sf_circle)
    ax.add_patch(sf_border)

    # ── Departure threshold ────────────────────────────────────────────────────
    dep_circle = plt.Circle((sx, sy), MIN_DEPARTURE_DIST,
                             fill=False, edgecolor='#ffaa00',
                             linewidth=1.5, linestyle=':', zorder=4)
    ax.add_patch(dep_circle)

    # ── Spawn point ────────────────────────────────────────────────────────────
    ax.plot(sx, sy, 'o', color='#00ff88', markersize=10, zorder=5)
    # Arrow showing spawn heading (theta=0 → pointing right)
    arrow_len = 0.12
    dx = arrow_len * np.cos(SPAWN["theta"])
    dy = arrow_len * np.sin(SPAWN["theta"])
    ax.annotate("", xy=(sx+dx, sy+dy), xytext=(sx, sy),
                arrowprops=dict(arrowstyle="->", color='#00ff88', lw=2.5))

    # ── Labels ─────────────────────────────────────────────────────────────────
    ax.text(sx + START_FINISH_RADIUS + 0.03, sy,
            f"Start/Finish\nr = {START_FINISH_RADIUS*100:.0f} cm",
            color='#00ff88', fontsize=9, va='center')

    ax.text(sx + MIN_DEPARTURE_DIST + 0.03, sy - 0.08,
            f"Min departure\nr = {MIN_DEPARTURE_DIST*100:.0f} cm",
            color='#ffaa00', fontsize=8, va='center')

    # ── Info box ───────────────────────────────────────────────────────────────
    info = (f"Spawn:  ({sx:.2f}, {sy:.2f})  θ={np.degrees(SPAWN['theta']):.0f}°\n"
            f"Max lateral error:  {MAX_LATERAL_ERROR*1000:.0f} mm\n"
            f"Max lap time:       {MAX_LAP_TIME:.0f} s")

    ax.text(0.02, 0.98, info, transform=ax.transAxes,
            fontsize=9, color='white', va='top', ha='left',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#2a2a4a',
                      edgecolor='#555577', alpha=0.9))

    # ── Legend ─────────────────────────────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor='#00ff88', alpha=0.4, label='Start/Finish zone'),
        mpatches.Patch(facecolor='none',
                       edgecolor='#ffaa00', linestyle=':', label='Min departure radius'),
        mpatches.Patch(facecolor='none',
                       edgecolor='#00ff88', label='Spawn point / heading'),
    ]
    ax.legend(handles=legend_handles, loc='lower right',
              facecolor='#2a2a4a', edgecolor='#555577',
              labelcolor='white', fontsize=9)

    # ── Axes ───────────────────────────────────────────────────────────────────
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_xlabel('x (m)', color='white')
    ax.set_ylabel('y (m)', color='white')
    ax.tick_params(colors='white')
    for spine in ax.spines.values():
        spine.set_edgecolor('#555577')

    ax.set_title('bane_fase2  —  Lap Optimizer Track Preview',
                 color='white', fontsize=13, fontweight='bold', pad=12)

    plt.tight_layout()

    # Save
    out_path = os.path.join(_OUTPUT_DIR, 'track_preview.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    print(f"✅ Saved: output/track_preview_{ts}.png")

    plt.show()


if __name__ == "__main__":
    preview()

