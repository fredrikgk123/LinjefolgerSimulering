# track_generator.py

import numpy as np
from PIL import Image, ImageDraw
from config import *

def generate_default_track():
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