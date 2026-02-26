# qtr_array.py — simulates the QTRX-HD-25RC sensor array (9 channels, 4 mm pitch, RC mode).
import numpy as np
from config import *
from physics.robot_model import world_to_pixel, bilinear_sample

class QTRArray:
    def __init__(self):
        # Sensor lateral positions in robot body frame, centred at y=0.
        positions_y = (np.arange(QTR_CHANNELS) - (QTR_CHANNELS - 1) / 2) * QTR_SPACING_M
        self.sensor_pos_body = np.stack([
            np.full(QTR_CHANNELS, QTR_SENSOR_OFFSET_M),
            positions_y
        ], axis=1)

    def read(self, robot, map_blur):
        R = robot.rotation_matrix()
        sensor_xy_world = (R @ self.sensor_pos_body.T).T + robot.position

        readings = []
        for (sx, sy) in sensor_xy_world:
            px, py = world_to_pixel(sx, sy)
            value = bilinear_sample(map_blur, px, py) / 255.0
            readings.append(1.0 - value)   # 0 = white, 1 = black line

        arr   = np.array(readings)
        noisy = np.clip(arr + np.random.randn(QTR_CHANNELS) * QTR_NOISE_STD, 0, 1)

        # Quantize to QTR_ADC_BITS (matches 0–1000 calibrated range of Pololu library)
        max_val = (1 << QTR_ADC_BITS) - 1
        return np.round(noisy * max_val) / max_val
