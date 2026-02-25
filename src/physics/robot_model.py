# robot_model.py

import numpy as np
from config import *
from physics.friction import slip_ratio, friction_force

def bilinear_sample(arr, x, y):
    x0 = int(np.floor(x)); y0 = int(np.floor(y))
    x1 = min(x0+1, arr.shape[1]-1)
    y1 = min(y0+1, arr.shape[0]-1)
    dx = x - x0; dy = y - y0

    v00 = arr[y0,x0]; v10 = arr[y0,x1]
    v01 = arr[y1,x0]; v11 = arr[y1,x1]

    return (1-dx)*(1-dy)*v00 + dx*(1-dy)*v10 + (1-dx)*dy*v01 + dx*dy*v11

def world_to_pixel(x, y):
    W,H = MAP_SIZE_M
    px = np.clip(x * PX_PER_METER, 0, int(W*PX_PER_METER)-1)
    py = np.clip((H - y) * PX_PER_METER, 0, int(H*PX_PER_METER)-1)
    return px, py

class Robot:
    def __init__(self):
        W, H = MAP_SIZE_M
        self.position = np.array([0.05 * W, 0.5 * H])  # default: track start centre
        self.theta = 0.0

        self.vL = 0.0
        self.vR = 0.0
        self.mass = ROBOT_MASS

    def rotation_matrix(self):
        c,s = np.cos(self.theta), np.sin(self.theta)
        return np.array([[c, -s],[s, c]])

    def update(self, vL_cmd, vR_cmd):
        # motor first-order dynamics
        self.vL += (vL_cmd - self.vL) * (DT / MOTOR_TAU)
        self.vR += (vR_cmd - self.vR) * (DT / MOTOR_TAU)

        # saturate
        self.vL = np.clip(self.vL, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        self.vR = np.clip(self.vR, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

        # slip model
        slip_L = slip_ratio(vL_cmd, self.vL)
        slip_R = slip_ratio(vR_cmd, self.vR)
        muL = friction_force(slip_L)
        muR = friction_force(slip_R)

        v_L_eff = self.vL * (1 - 0.1*muL)
        v_R_eff = self.vR * (1 - 0.1*muR)

        v = 0.5 * (v_L_eff + v_R_eff)
        w = (v_R_eff - v_L_eff) / WHEEL_BASE

        # integrate
        self.position += np.array([np.cos(self.theta), np.sin(self.theta)]) * v * DT
        self.theta += w * DT
