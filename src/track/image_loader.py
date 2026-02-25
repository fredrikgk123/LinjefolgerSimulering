# image_loader.py
# Universal loader for any top-down track image (black line on white background).
# Drop your image into assets/ and run: python3 main.py --track ../assets/my_image.png

import numpy as np
from PIL import Image
from config import *


def load_track_image(path):
    """
    Load a top-down track image and resize it to the simulator resolution.

    Requirements for the image:
      - Top-down view of the track
      - Dark line on a light background (or light line on dark — will be auto-inverted)
      - Any common format: PNG, JPG, BMP, etc.

    Args:
        path: Absolute or relative path to the image file

    Returns:
        PIL Image (grayscale, L mode): black line (0) on white background (255),
        sized to (W * PX_PER_METER) x (H * PX_PER_METER)
    """
    W, H = MAP_SIZE_M

    img = Image.open(path).convert("L")

    # Auto-invert if the image has a dark background with a light line
    arr = np.array(img)
    if arr.mean() < 128:
        img = Image.fromarray(255 - arr)

    # Resize to simulator resolution
    img = img.resize(
        (int(W * PX_PER_METER), int(H * PX_PER_METER)),
        resample=Image.LANCZOS
    )

    return img

