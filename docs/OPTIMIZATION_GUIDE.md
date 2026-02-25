# PID Parameter Optimization Guide

## Overview

An automated machine learning system that tests hundreds of PID parameter combinations and finds the **optimal configuration** for your robot across all track types.

## How It Works

### Two Optimization Methods:

1. **Grid Search** (Exhaustive)
   - Tests every combination in a defined parameter space
   - Guarantees finding the global optimum
   - Slower but most reliable
   - Best for finding the absolute best tuning

2. **Bayesian Optimization** (Random Sampling)
   - Randomly samples from parameter space
   - Faster than grid search
   - Good for quickly finding good solutions
   - Best for fast iteration/refinement

## Quick Start

### Run Quick Optimization (Fastest - ~13 minutes)
```bash
python3 optimize.py --mode quick
```

Tests ~27 parameter combinations across all 5 tracks.

### Run Full Optimization (Slowest - ~50+ hours)
```bash
python3 optimize.py --mode full
```

Tests ~6,400 combinations for maximum accuracy.

### Run Bayesian Optimization (Medium - ~15 minutes)
```bash
python3 optimize.py --mode bayesian --iterations 30
```

Tests 30 random combinations from large space.

## What Gets Tested

### Parameters Optimized:

**PID Gains:**
- `Kp` (Proportional): 60-130
- `Ki` (Integral): 2.5-5.5
- `Kd` (Derivative): 15-25
- `limit` (Max turn rate): 16-24

**Speed Controller:**
- `straight_speed`: 0.80-1.0 m/s
- `turn_speed`: 0.55-0.80 m/s

### Evaluation Metric:

Tests each combination on **all 5 tracks** and uses **Average RMS Error** as the fitness metric:
- Lower RMS = Better tracking quality
- Tests generalization across different geometries

## Output

### Results Saved:
- `optimization_results.json` - Complete results data
- `multi_track_results.png` - Best configuration visualizations
- `multi_track_bars.png` - Bar chart comparisons
- Console output - Best parameters in code format

### Console Output Shows:

```
[27] Testing: Kp=110.0, Ki=4.2, Kd=20.5, Speed=0.95
   ✨ NEW BEST! RMS Error: 8.43mm

🏆 BEST PARAMETERS FOUND:
  Kp: 95.30
  Ki: 4.15
  Kd: 21.20
  Limit: 20.00
  Straight Speed: 0.92
  Turn Speed: 0.68

📊 BEST PERFORMANCE:
  Average RMS Error: 8.43mm
  Average Max Error: 15.27mm
  Average Settling Time: 0.52s

USE THESE PARAMETERS IN YOUR CODE:
pid = PID(kp=95.3, ki=4.15, kd=21.2, ...)
```

## Usage Examples

### Example 1: Find Best PID Quickly
```bash
python3 optimize.py --mode quick
# Wait ~13 minutes
# Copy printed parameters into main.py
```

### Example 2: Refine Existing Tuning
```bash
# Modify param_ranges in optimize.py to be narrower around current best
# Re-run quick optimization
```

### Example 3: Exhaustive Search
```bash
python3 optimize.py --mode full
# Wait 50+ hours for absolute optimal tuning
# Great for publication/competition
```

## Performance Expectations

| Mode | Combinations | Time | Accuracy |
|------|-------------|------|----------|
| Quick | 27 | ~13 min | Good |
| Bayesian | 30-100 | ~15-50 min | Very Good |
| Full | 6,400+ | 50+ hours | Excellent |

## Advanced Usage

### Custom Parameter Ranges

Edit `pid_optimizer.py` and modify `PIDOptimizer.__init__()`:

```python
param_ranges = {
    'kp': np.linspace(80, 120, 5),    # 5 values between 80-120
    'ki': np.linspace(3, 5, 4),       # 4 values between 3-5
    'kd': np.linspace(18, 24, 4),     # etc...
    'limit': np.linspace(18, 22, 3),
    'straight_speed': np.linspace(0.85, 1.0, 3),
    'turn_speed': np.linspace(0.60, 0.75, 3),
}
```

### Programmatic Usage

```python
from pid_optimizer import PIDOptimizer
import numpy as np

# Define parameter ranges
param_ranges = {
    'kp': np.linspace(60, 130, 5),
    'ki': np.linspace(2.5, 5.5, 5),
    # ... etc
}

# Create optimizer
optimizer = PIDOptimizer(param_ranges=param_ranges)

# Run optimization
optimizer.grid_search()

# Get results
best_params = optimizer.best_params
best_rms = optimizer.best_result['avg_rms_error']

print(f"Best Kp: {best_params['kp']}")
print(f"Best RMS Error: {best_rms*1000:.2f}mm")
```

## Tips for Best Results

1. **Start with Quick Mode**
   - Get baseline understanding of parameter space
   - Takes only ~13 minutes
   - Good enough for most applications

2. **Refine Around Best Result**
   - After quick mode, modify ranges to be narrower
   - Re-run quick mode
   - Converges to better solution

3. **Use Bayesian for Refinement**
   - After identifying general good range
   - Faster than full grid search
   - Good for fine-tuning

4. **Save Your Results**
   - Results are auto-saved as JSON
   - Good for comparing multiple optimization runs
   - Track improvements over time

## Performance Targets

After optimization, you should achieve:

| Metric | Target |
|--------|--------|
| Average RMS Error | < 10mm |
| Average Max Error | < 18mm |
| Worst Track | < 20mm RMS |
| Generalization | Works on all 5 tracks |

## Understanding Results

### Good Optimization Result:
```
All tracks: ✓ GOOD to ✓ EXCELLENT
Average RMS Error: ~8-12mm
Consistent across different geometries
```

### Poor Optimization Result:
```
Some tracks: ⚠ OK or ✗ POOR
Average RMS Error: > 15mm
Huge variation between tracks
→ Increase search space and try again
```

## Workflow

1. **Run quick optimization**
   ```bash
   python3 optimize.py --mode quick
   ```

2. **Copy best parameters to `main.py`**

3. **Test visually with `python3 main.py`**

4. **If happy, done! If not:**
   - Narrow param_ranges around best result
   - Run quick mode again
   - Repeat

5. **For final tuning:**
   - Run Bayesian with more iterations
   - Or full grid search for publication-quality

## Time Estimates

Based on 0.5 minutes per track test (5 tracks = 2.5 min per combination):

- **Quick (27 combos)**: ~13 minutes
- **Bayesian 30**: ~75 minutes
- **Bayesian 100**: ~250 minutes (4 hours)
- **Full (6,400 combos)**: ~260 hours (10+ days)

## Notes

- ✅ Runs on all 5 tracks automatically
- ✅ Saves results to JSON for analysis
- ✅ Generates visualizations automatically
- ✅ Shows top 5 results
- ✅ Prints code-ready parameters
- ⚠️ Full mode takes days - only if you have time
- 💡 Quick mode is usually sufficient

The system will find a **general-purpose PID** that works well across all track types, not just optimized for one! 🎯

