# image_loader.py — load any top-down track image (black line on white background).

import os
import numpy as np
from PIL import Image
from config import *

_SUPPORTED_EXTENSIONS = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.tif']


def load_track_image(path):
    """
    Load a track image, auto-invert if dark-background, resize to simulator resolution.
    Extension is optional — will search for a match if omitted.
    """
    W, H = MAP_SIZE_M

    if not os.path.splitext(path)[1]:
        for ext in _SUPPORTED_EXTENSIONS:
            candidate = path + ext
            if os.path.exists(candidate):
                path = candidate
                break
        else:
            raise FileNotFoundError(
                f"No track image found at '{path}' with extensions {_SUPPORTED_EXTENSIONS}.")

    img = Image.open(path).convert("L")

    arr = np.array(img)
    if arr.mean() < 128:   # dark background → invert
        img = Image.fromarray(255 - arr)

    img = img.resize(
        (int(W * PX_PER_METER), int(H * PX_PER_METER)),
        resample=Image.LANCZOS)

    return img

