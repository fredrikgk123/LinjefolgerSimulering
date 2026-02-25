# multi_track.py
"""
Multiple track generators for testing PID controller robustness.
"""

import os
import numpy as np
from PIL import Image, ImageDraw
from config import MAP_SIZE_M, PX_PER_METER, TRACK_WIDTH_M
from track.image_loader import load_track_image

_ASSETS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', 'assets'))


def generate_sine_track():
    """Original sine wave track."""
    W, H = MAP_SIZE_M
    px_w = int(W * PX_PER_METER)
    px_h = int(H * PX_PER_METER)

    img = Image.new("L", (px_w, px_h), 255)
    draw = ImageDraw.Draw(img)

    xs = np.linspace(0.05*W, 0.95*W, 1200)
    ys = 0.5*H + 0.25*H*np.sin(3*xs/W*2*np.pi)

    pts = [(int(x*PX_PER_METER), px_h - int(y*PX_PER_METER)) for x,y in zip(xs,ys)]
    thickness = int(TRACK_WIDTH_M * PX_PER_METER)
    draw.line(pts, fill=0, width=thickness)

    return img


def generate_tight_sine_track():
    """Tighter sine wave - sharper turns."""
    W, H = MAP_SIZE_M
    px_w = int(W * PX_PER_METER)
    px_h = int(H * PX_PER_METER)

    img = Image.new("L", (px_w, px_h), 255)
    draw = ImageDraw.Draw(img)

    xs = np.linspace(0.05*W, 0.95*W, 1200)
    # More oscillations, tighter curves
    ys = 0.5*H + 0.20*H*np.sin(5*xs/W*2*np.pi)

    pts = [(int(x*PX_PER_METER), px_h - int(y*PX_PER_METER)) for x,y in zip(xs,ys)]
    thickness = int(TRACK_WIDTH_M * PX_PER_METER)
    draw.line(pts, fill=0, width=thickness)

    return img


def generate_wide_sine_track():
    """Wider sine wave - gentler curves."""
    W, H = MAP_SIZE_M
    px_w = int(W * PX_PER_METER)
    px_h = int(H * PX_PER_METER)

    img = Image.new("L", (px_w, px_h), 255)
    draw = ImageDraw.Draw(img)

    xs = np.linspace(0.05*W, 0.95*W, 1200)
    # Fewer oscillations, wider curves
    ys = 0.5*H + 0.30*H*np.sin(2*xs/W*2*np.pi)

    pts = [(int(x*PX_PER_METER), px_h - int(y*PX_PER_METER)) for x,y in zip(xs,ys)]
    thickness = int(TRACK_WIDTH_M * PX_PER_METER)
    draw.line(pts, fill=0, width=thickness)

    return img


def generate_straight_track():
    """Straight line - test steady-state speed."""
    W, H = MAP_SIZE_M
    px_w = int(W * PX_PER_METER)
    px_h = int(H * PX_PER_METER)

    img = Image.new("L", (px_w, px_h), 255)
    draw = ImageDraw.Draw(img)

    # Straight line down the middle
    y_center = 0.5 * H
    pts = [(0, px_h - int(y_center*PX_PER_METER)),
           (px_w, px_h - int(y_center*PX_PER_METER))]
    thickness = int(TRACK_WIDTH_M * PX_PER_METER)
    draw.line(pts, fill=0, width=thickness)

    return img


def generate_s_curve_track():
    """S-curve - test rapid direction changes."""
    W, H = MAP_SIZE_M
    px_w = int(W * PX_PER_METER)
    px_h = int(H * PX_PER_METER)

    img = Image.new("L", (px_w, px_h), 255)
    draw = ImageDraw.Draw(img)

    xs = np.linspace(0.05*W, 0.95*W, 1200)
    # S-shaped curve
    ys = 0.5*H + 0.25*H*np.tanh(4*xs/W - 2)

    pts = [(int(x*PX_PER_METER), px_h - int(y*PX_PER_METER)) for x,y in zip(xs,ys)]
    thickness = int(TRACK_WIDTH_M * PX_PER_METER)
    draw.line(pts, fill=0, width=thickness)

    return img


def generate_bane_fase2_track():
    """Real competition track from bane_fase2.png."""
    return load_track_image(os.path.join(_ASSETS_DIR, 'bane_fase2.png'))


# Available tracks
ALL_TRACKS = {
    "sine":        generate_sine_track,
    "tight_sine":  generate_tight_sine_track,
    "wide_sine":   generate_wide_sine_track,
    "straight":    generate_straight_track,
    "s_curve":     generate_s_curve_track,
    "bane_fase2":  generate_bane_fase2_track,
}


def get_track_names():
    """Return list of available track names."""
    return list(ALL_TRACKS.keys())


def get_track(name):
    """Get track image by name."""
    if name not in ALL_TRACKS:
        raise ValueError(f"Unknown track: {name}. Available: {list(ALL_TRACKS.keys())}")
    return ALL_TRACKS[name]()

