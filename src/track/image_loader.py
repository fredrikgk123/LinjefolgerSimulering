# image_loader.py
# Universal loader for any top-down track image (black line on white background).
# Drop your image into assets/ and run: python3 main.py --track ../assets/my_image.png

import os
import numpy as np
from PIL import Image
from config import *

_SUPPORTED_EXTENSIONS = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.tif']


def load_track_image(path):
    """
    Load a top-down track image and resize it to the simulator resolution.
    If the path has no extension, automatically searches for a matching file.

    Requirements for the image:
      - Top-down view of the track
      - Dark line on a light background (light on dark is auto-inverted)
      - Any common format: PNG, JPG, BMP, TIFF, etc.

    Args:
        path: Absolute or relative path to the image file (extension optional)

    Returns:
        PIL Image (grayscale, L mode): black line (0) on white background (255),
        sized to (W * PX_PER_METER) x (H * PX_PER_METER)
    """
    W, H = MAP_SIZE_M

    # If no extension given, search for a matching file
    if not os.path.splitext(path)[1]:
        for ext in _SUPPORTED_EXTENSIONS:
            candidate = path + ext
            if os.path.exists(candidate):
                path = candidate
                break
        else:
            raise FileNotFoundError(
                f"No track image found at '{path}' with extensions "
                f"{_SUPPORTED_EXTENSIONS}. Check the filename in assets/."
            )

    img = Image.open(path).convert("L")

    # Auto-invert if image has a dark background with a light line
    arr = np.array(img)
    if arr.mean() < 128:
        img = Image.fromarray(255 - arr)

    # Resize to simulator resolution
    img = img.resize(
        (int(W * PX_PER_METER), int(H * PX_PER_METER)),
        resample=Image.LANCZOS
    )

    return img

