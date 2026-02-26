# friction.py — Pacejka-like tyre slip model for sponge-rubber on vinyl.
import numpy as np
from config import *

# Fraction of speed lost per unit friction force (calibrated for 69% wheel load).
SLIP_INFLUENCE = 0.35

def slip_ratio(v_cmd, v_actual):
    V = max(abs(v_cmd), 1e-4)
    return (v_cmd - v_actual) / V

def friction_force(slip):
    # Simplified Pacejka curve: sharp saturation at k=8 (sponge-rubber on vinyl)
    return MU_SLIDE * np.tanh(8 * slip)
