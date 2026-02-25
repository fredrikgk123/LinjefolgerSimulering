# qtr_array.py

import numpy as np
from config import *
from physics.robot_model import world_to_pixel, bilinear_sample

class QTRArray:
    def __init__(self):
        positions_y = (np.arange(QTR_CHANNELS) - (QTR_CHANNELS-1)/2) * QTR_SPACING_M
        self.sensor_pos_body = np.stack([
            np.full(QTR_CHANNELS, QTR_SENSOR_OFFSET_M),
            positions_y
        ], axis=1)

    def read(self, robot, map_blur):
        # Transform sensor positions into world coordinates
        R = robot.rotation_matrix()
        sensor_xy_world = (R @ self.sensor_pos_body.T).T + robot.position

        readings = []
        for (sx, sy) in sensor_xy_world:
            px, py = world_to_pixel(sx, sy)
            value = bilinear_sample(map_blur, px, py) / 255.0
            line_strength = 1.0 - value
            readings.append(line_strength)

        arr = np.array(readings)
        noisy = np.clip(arr + np.random.randn(QTR_CHANNELS)*QTR_NOISE_STD, 0, 1)

        # Quantize
        max_val = (1 << QTR_ADC_BITS) - 1
        quant = np.round(noisy * max_val) / max_val

        return quant
