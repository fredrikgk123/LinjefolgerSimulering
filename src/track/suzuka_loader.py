# suzuka_loader.py

from PIL import Image
from config import *

def load_suzuka(path):
    img = Image.open(path).convert("L")
    W, H = MAP_SIZE_M

    # Resize to correct scale
    img = img.resize((int(W*PX_PER_METER), int(H*PX_PER_METER)))

    return img