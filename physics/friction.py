# friction.py
import numpy as np
from config import *

def slip_ratio(v_cmd, v_actual):
    # Longitudinal slip (simple model)
    V = max(abs(v_cmd), 1e-4)
    return (v_cmd - v_actual) / V

def friction_force(slip):
    # Pacejka-like simplified tire friction curve
    mu = MU_SLIDE * np.tanh(8 * slip)
    return mu