# Multi-Track Testing System

## Overview

A system to test your PID controller across **multiple different track geometries** at once, so you can tune a general-purpose controller instead of one optimized only for the sine wave.

## Available Tracks

| Track Name | Description | Use Case |
|------------|-------------|----------|
| **sine** | Original sine wave (3 oscillations) | General testing |
| **tight_sine** | Tight curves (5 oscillations) | Tests sharp turn response |
| **wide_sine** | Gentle curves (2 oscillations) | Tests high-speed stability |
| **straight** | Perfectly straight line | Tests steady-state speed |
| **s_curve** | S-shaped rapid direction change | Tests transition ability |

## How to Use

### Option 1: Run All Tracks at Once
```bash
cd tests
python3 test_multi_track.py
```

Output example:
```
[1/5] Running on 'sine' track...
      Max Error: 15.23mm | RMS Error: 8.45mm | Settle Time: 0.50s
[2/5] Running on 'tight_sine' track...
      Max Error: 22.15mm | RMS Error: 12.30mm | Settle Time: 0.75s
...

AGGREGATED RESULTS
==================
Average Max Error:        18.54mm
Average RMS Error:        10.23mm
Best performing track:    sine (15.23mm)
Worst performing track:   tight_sine (22.15mm)
```

### Option 2: Run Specific Tracks in Your Code
```python
from multi_track_simulator import run_multi_track_test
from control.pid_controller import PID, SpeedController

pid = PID(kp=100, ki=3.5, kd=16)
speed_controller = SpeedController()

# Test only certain tracks
results = run_multi_track_test(
    pid, speed_controller,
    tracks=['sine', 'tight_sine', 's_curve'],
    show_viz=True  # Show visualization for first track
)
```

### Option 3: Manual Per-Track Testing
```python
from multi_track_simulator import run_simulation_on_track

metrics, traj, err_log, sensor_log, blur_arr = run_simulation_on_track(
    track_name='tight_sine',
    pid=pid,
    speed_controller=speed_controller,
    show_visualization=True
)

print(f"Max error on tight_sine: {metrics['max_error']*1000:.2f}mm")
```

## Understanding Results

### Key Metrics:

- **Average Max Error**: How high does error spike? (lower = better)
- **Average RMS Error**: Steady tracking quality (lower = better)
- **Settling Time**: How fast to recover from perturbations
- **Worst Track**: Which track breaks your tuning? (points to gaps)
- **Best Track**: Which is your strength?

### Interpretation:

```
IF average_rms_error < 10mm:  ✅ EXCELLENT - Generalizes well
IF average_rms_error < 15mm:  ✅ GOOD - Solid general controller
IF average_rms_error < 25mm:  ⚠️ OK - Works but not great
IF average_rms_error > 25mm:  ❌ POOR - Needs retuning
```

### Finding Gaps:

If your PID performs well on `sine` but poorly on `tight_sine`, it means:
- Your Kp is too low for sharp turns → Increase Kp slightly
- Your Kd is too low for fast changes → Increase Kd

## Tuning Strategy

### Step 1: Test Current PID
```bash
python3 tests/test_multi_track.py
```
Note which track performs worst.

### Step 2: Adjust for Weak Points
- **Worst track is tight curves?** → Increase Kp and Kd
- **Worst track is straight line?** → Increase speed_controller.straight_speed
- **Too much overshoot?** → Increase Kd more
- **Oscillating?** → Decrease Kp or increase Kd

### Step 3: Iterate
Change one parameter, test again, check if average improves.

## Example: Improving Your Current PID

Your current setup:
```python
pid = PID(kp=100.0, ki=3.5, kd=16.0, limit=18.0)
```

If tight_sine performs worst, try:
```python
pid = PID(kp=110.0, ki=3.5, kd=18.0, limit=20.0)  # 10% more Kp, Kd
```

Then test again and compare aggregated metrics.

## Implementation Details

### Files:
- **`track/multi_track.py`** - 5 different track generators
- **`multi_track_simulator.py`** - Run simulations and aggregate results
- **`tests/test_multi_track.py`** - Quick test script

### Key Functions:

```python
# Run all available tracks
run_multi_track_test(pid, speed_controller)

# Run specific tracks
run_multi_track_test(pid, speed_controller, 
                     tracks=['sine', 'tight_sine', 's_curve'])

# Test one track manually
run_simulation_on_track('tight_sine', pid, speed_controller)
```

## Benefits

✅ **Find weak points** - See which track types your PID struggles with  
✅ **Generalize better** - Tune for average performance, not single case  
✅ **Faster iteration** - Run 5 tracks in ~1 minute instead of manual testing  
✅ **Objective metrics** - Real numbers instead of visual guessing  
✅ **Confidence** - Know your controller works on multiple geometries  

## Quick Start

```bash
# 1. Test your current PID on all tracks
python3 tests/test_multi_track.py

# 2. Note the worst performing track
# 3. Adjust one PID parameter based on the weakness
# 4. Repeat until average RMS error < 12mm
```

That's it! You now have a data-driven way to tune a general-purpose PID controller. 🎯

