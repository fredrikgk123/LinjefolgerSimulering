#!/usr/bin/env python3
"""
Test if matplotlib setup consumes random numbers.
"""

import numpy as np
from config import *

print("Test 1: Check if numpy random state is identical after setting seed")
print("="*70)

# Test A: Just set seed
np.random.seed(NOISE_SEED)
state_A = np.random.get_state()
random_A = [np.random.randn() for _ in range(5)]
print(f"Test A (just seed): first 5 random numbers = {random_A}")

# Test B: Set seed, then import matplotlib and create figure
np.random.seed(NOISE_SEED)
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

state_B_before = np.random.get_state()
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)
ax.plot([0, 1], [0, 1])
plt.close(fig)
state_B_after = np.random.get_state()

random_B = [np.random.randn() for _ in range(5)]
print(f"Test B (with matplotlib): first 5 random numbers = {random_B}")

# Compare
if random_A == random_B:
    print("\n✓ IDENTICAL: Matplotlib does NOT consume random numbers")
else:
    print("\n✗ DIFFERENT: Matplotlib DOES consume random numbers!")
    print("  This would cause desynchronization between optimizer and visualization")

print("="*70)

