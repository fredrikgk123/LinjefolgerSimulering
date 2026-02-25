# multi_track.py
"""
Track registry for the multi-track simulator and optimizer.
Contains the original sine-wave track plus any image-based tracks from assets/.
"""

import os
from PIL import Image, ImageDraw
import numpy as np
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

    pts = [(int(x*PX_PER_METER), px_h - int(y*PX_PER_METER)) for x, y in zip(xs, ys)]
    draw.line(pts, fill=0, width=int(TRACK_WIDTH_M * PX_PER_METER))
    return img


def _load_asset(filename):
    """Load a track image from assets/."""
    return load_track_image(os.path.join(_ASSETS_DIR, filename))


# ── Track registry ────────────────────────────────────────────────────────────
# Add new image-based tracks here: "name": lambda: _load_asset("filename.png")
ALL_TRACKS = {
    "sine":       generate_sine_track,
    "bane_fase2": lambda: _load_asset("bane_fase2.png"),
    "suzuka":     lambda: _load_asset("suzuka.png"),
}


def get_track_names():
    return list(ALL_TRACKS.keys())


def get_track(name):
    if name not in ALL_TRACKS:
        raise ValueError(f"Unknown track: '{name}'. Available: {list(ALL_TRACKS.keys())}")
    return ALL_TRACKS[name]()
