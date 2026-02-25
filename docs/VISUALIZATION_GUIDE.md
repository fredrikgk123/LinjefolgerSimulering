# Multi-Track Visualization Guide

## Overview

Three different visualization formats to analyze your PID controller's performance across multiple tracks:

1. **Stacked List Format** - All tracks in a vertical list with their metrics
2. **Comparison Bar Charts** - Side-by-side comparison of key metrics
3. **Summary Table** - Comprehensive table view with aggregated stats

## Usage

### Run All Visualizations at Once

```bash
cd tests
python3 test_multi_track.py
```

This will:
1. Run simulations on all 5 tracks
2. Generate 3 visualization images:
   - `multi_track_results.png` - Stacked list format
   - `multi_track_bars.png` - Bar chart comparisons
   - `multi_track_table.png` - Summary table

### Use Programmatically

```python
from multi_track_simulator import run_multi_track_test
from multi_track_plots import (
    plot_multi_track_results,
    plot_multi_track_comparison_bars,
    plot_multi_track_summary_table
)

# Run tests
results = run_multi_track_test(pid, speed_controller)

# Generate visualizations
plot_multi_track_results(results)
plot_multi_track_comparison_bars(results)
plot_multi_track_summary_table(results)
```

## Visualization Formats

### 1. Stacked List Format (`multi_track_results.png`)

Shows all tracks stacked vertically with:
- **Track name** (left)
- **Status indicator** (colored box showing EXCELLENT/GOOD/OK/POOR)
- **All metrics** (Max error, RMS error, Mean error, Settling time, Steady-state)

**Best for:** Quick overview of all tracks at once

Example:
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SINE         ✓ EXCELLENT    Max: 12.5mm | RMS: 7.2mm | ...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  TIGHT_SINE   ⚠ OK           Max: 18.3mm | RMS: 11.5mm | ...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  WIDE_SINE    ✓ GOOD         Max: 14.2mm | RMS: 8.9mm | ...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 2. Comparison Bar Charts (`multi_track_bars.png`)

Three side-by-side bar charts:
- **Max Error** - Spike magnitude on each track
- **RMS Error** - Overall tracking quality (most important)
- **Settling Time** - Recovery speed

**Best for:** Comparing performance across specific metrics

**How to read:**
- Shorter bars = better performance
- RMS Error is most important (tracking quality)
- Max Error shows worst-case spike

### 3. Summary Table (`multi_track_table.png`)

Comprehensive table with:
- All tracks listed as rows
- Columns: Max, RMS, Mean, Settling Time, Steady-State
- **AVERAGE** row showing aggregated performance
- Color-coded rows (alternating light/dark)

**Best for:** Detailed analysis and record-keeping

## Metrics Explained

| Metric | Unit | What It Means | Good Value |
|--------|------|---------------|------------|
| **Max Error** | mm | Highest deviation from line | < 20mm |
| **RMS Error** | mm | Overall tracking quality (root mean square) | < 12mm |
| **Mean Error** | mm | Average absolute deviation | < 8mm |
| **Settling Time** | s | Time to recover from disturbance | < 1.0s |
| **Steady-State** | mm | Error during normal operation | < 5mm |

## Color Coding

### Status Colors (Stacked List):
- 🟢 **Green (✓ EXCELLENT)** - RMS Error < 10mm
- 🔵 **Blue (✓ GOOD)** - RMS Error < 15mm
- 🟠 **Orange (⚠ OK)** - RMS Error < 25mm
- 🔴 **Red (✗ POOR)** - RMS Error > 25mm

### Bar Chart Colors:
- Each track gets a unique color
- Bar height = metric magnitude

### Table Colors:
- 🔷 **Dark header** - Column names
- 🔵 **Blue aggregate row** - AVERAGE across all tracks
- ⚪ **White/gray rows** - Individual track data

## Interpretation Examples

### Example 1: Well-Tuned Controller
```
sine:           Max: 12mm  | RMS: 7mm   ✓ EXCELLENT
tight_sine:     Max: 15mm  | RMS: 9mm   ✓ EXCELLENT
wide_sine:      Max: 11mm  | RMS: 6mm   ✓ EXCELLENT
AVERAGE:        Max: 13mm  | RMS: 7.3mm ✓ GOOD overall
```
→ **Conclusion**: PID generalizes well across geometries

### Example 2: Struggles with Tight Turns
```
sine:           Max: 12mm  | RMS: 7mm   ✓ EXCELLENT
tight_sine:     Max: 35mm  | RMS: 22mm  ⚠ OK
wide_sine:      Max: 11mm  | RMS: 6mm   ✓ EXCELLENT
AVERAGE:        Max: 19mm  | RMS: 11.7mm ⚠ OK overall
```
→ **Conclusion**: Increase Kp and Kd for sharper turns

### Example 3: Oscillating
```
sine:           Max: 28mm  | RMS: 18mm  ⚠ OK
tight_sine:     Max: 35mm  | RMS: 24mm  ⚠ OK
wide_sine:      Max: 25mm  | RMS: 16mm  ⚠ OK
AVERAGE:        Max: 29mm  | RMS: 19.3mm ⚠ OK overall
```
→ **Conclusion**: Decrease Kp or increase Kd to add damping

## Using for Tuning Iteration

### Step 1: Generate Baseline
```bash
python3 tests/test_multi_track.py
# Saves: multi_track_results.png, multi_track_bars.png, multi_track_table.png
```

### Step 2: Identify Weak Points
Look at the stacked list - which track(s) have ⚠ or ✗?

### Step 3: Adjust PID
Based on worst track:
- **Tight curves failing?** → Increase Kp/Kd
- **Oscillating?** → Decrease Kp, increase Kd
- **Too slow on straights?** → Increase speed_controller.straight_speed

### Step 4: Test Again
```bash
python3 tests/test_multi_track.py
```
Compare new images with previous ones.

### Step 5: Iterate
Repeat until AVERAGE is ✓ GOOD (RMS Error < 15mm)

## Tips

✅ **Print the images** - Physical printouts help spot patterns  
✅ **Save iterations** - Rename images: `multi_track_v1.png`, `v2.png`, etc.  
✅ **Focus on AVERAGE** - Individual tracks can vary, average matters  
✅ **RMS > Max** - RMS Error is more important than Max Error  

## Output Files

All images saved in current directory:
- `multi_track_results.png` - Stacked list (high-level overview)
- `multi_track_bars.png` - Bar charts (metric comparison)
- `multi_track_table.png` - Summary table (detailed data)

Perfect for:
- 📊 Presentations
- 📝 Documentation
- 🔍 Analysis
- 📈 Tracking improvements over iterations

