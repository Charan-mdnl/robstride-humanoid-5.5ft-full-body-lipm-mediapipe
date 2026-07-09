# Motor System Identification via Bode Plot Analysis

This guide explains how to characterize your motor using multisine torque excitation and frequency response analysis.

## Overview

The workflow uses the C++ torque control test to generate multisine excitation, logs torque and velocity, then analyzes the frequency response to extract motor parameters:
- **I** — Inertia (kg⋅m²)
- **B** — Friction/Damping (Nm⋅s/rad)
- **K** — Stiffness (Nm/rad) — typically 0 for rigid joints

## Quick Start

### Step 1: Generate Test Data (C++)

Run the torque control test:
```bash
cd /home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/cpp/build
sudo ./multi_motor_functionality --hz 100 --can can0 t 5.0 11 rs04 10
```

**Arguments:**
- `t` — torque control mode
- `5.0` — duration in seconds
- `11` — motor ID
- `rs04` — motor type
- `10` — torque amplitude (Nm) for multisine

**Output:** Generates aligned CSV file with timestamp in filename:
```
motor_log_aligned_20260608_170004.csv
```

### Step 2: Analyze with Python Scripts

All scripts require the aligned CSV file. Navigate to build directory:
```bash
cd /home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/cpp/build
PYTHON="../../../venv/bin/python"
```

---

## Available Analysis Scripts

### 1. Generate Bode Plot

**Purpose:** Visualize frequency response magnitude and phase

**Command:**
```bash
$PYTHON ../examples/plot_bode.py motor_log_aligned_20260608_170004.csv
```

**Output:** `motor_log_aligned_20260608_170004_bode.png`

**Shows:**
- Top: Magnitude (dB) vs Frequency (Hz)
- Bottom: Phase (degrees) vs Frequency (Hz)
- Typical peak at low frequencies (1-5 Hz)

**Interpretation:**
- Flat magnitude at low freq → friction-dominated
- Roll-off at high freq → inertia-dominated

---

### 2. Estimate Inertia (3 Methods)

**Purpose:** Calculate inertia from multiple approaches

**Command:**
```bash
$PYTHON ../examples/estimate_inertia.py motor_log_aligned_20260608_170004.csv
```

**Output:** Console display (no file)

**Three methods:**
1. **Low-Frequency Average** (1-5 Hz range) — Most reliable
2. **Power-Law Fit** (log-log regression) — Theoretical estimate
3. **Peak-Based** (at resonance frequency) — Quick estimate

**Example output:**
```
METHOD 1: Low-Frequency Inertia Estimate
Frequency range: 1.95 - 4.88 Hz
Inertia estimate: 0.056665 ± 0.011689 kg⋅m²

METHOD 2: Power-Law Fit (Low-Frequency Region)
Log-log fit: log(|G|) = 0.3693 + -0.5602×log(f)
Estimated Inertia: 0.110010 kg⋅m²

METHOD 3: Peak-Based Estimate
Inertia estimate (from peak): 0.068019 kg⋅m²
```

---

### 3. Fit First-Order Model (I, B)

**Purpose:** Extract inertia and damping by fitting: `G(s) = 1/(I⋅s + B)`

**Command:**
```bash
$PYTHON ../examples/estimate_damping.py motor_log_aligned_20260608_170004.csv
```

**Output:** `motor_log_aligned_20260608_170004_fit.png` + console

**Fitted Parameters:**
- **I** (kg⋅m²) — Rotational inertia
- **B** (Nm⋅s/rad) — Viscous damping coefficient

**Example output:**
```
Inertia (I):        0.099407 kg⋅m²
Damping (B):        0.239749 Nm⋅s/rad
Corner frequency:   0.38 Hz (where B = I*ω)
DC gain (ω→0):      4.171037 (rad/s)/Nm

Friction torque at different speeds:
Velocity (rad/s)     Friction Torque (Nm)
1.0                  0.2397
5.0                  1.1987
10.0                 2.3975
```

**Physical interpretation:**
- Below 0.38 Hz: friction-limited behavior
- Above 0.38 Hz: inertia-limited behavior
- Friction = B × velocity

---

### 4. Compare Model Orders

**Purpose:** Determine if second-order (with stiffness) is needed

**Command:**
```bash
$PYTHON ../examples/compare_models.py motor_log_aligned_20260608_170004.csv
```

**Output:** `motor_log_aligned_20260608_170004_model_comparison.png` + console

**Compares:**
- **First-order:** `G(s) = 1/(I⋅s + B)`
- **Second-order:** `G(s) = 1/(I⋅s² + B⋅s + K)`

**Example output:**
```
MODEL COMPARISON
Model                    RMS Error (dB)    Improvement
First-order              6.3495 dB         —
Second-order             5.8759 dB         0.4736 dB (7.5%)
```

**Decision rule:**
- If improvement < 1 dB → Use **first-order** (simpler)
- If improvement > 2 dB → Use **second-order** (includes stiffness)

---

## Motor Characterization Example

### Test Setup
- Motor ID: 24
- Motor Type: RS04
- Multisine amplitude: 10 Nm
- Duration: 10 seconds
- Sampling rate: 1000 Hz

### Results (Motor 24)

| Parameter | Value | Unit | Meaning |
|-----------|-------|------|---------|
| **I** | 0.0994 | kg⋅m² | Rotational inertia |
| **B** | 0.240 | Nm⋅s/rad | Friction/damping coefficient |
| **K** | 0 | Nm/rad | No significant stiffness |

### System Model
$$\tau = I \alpha + B \omega$$
$$\tau = 0.0994 \cdot \alpha + 0.240 \cdot \omega$$

### Friction Examples
- At 57.3 deg/s (1 rad/s): 0.24 Nm friction
- At 286 deg/s (5 rad/s): 1.20 Nm friction
- At 573 deg/s (10 rad/s): 2.40 Nm friction

---

## Running All Scripts Together

**Batch script:**
```bash
#!/bin/bash
cd /home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/cpp/build
PYTHON="/home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/venv/bin/python"
CSV="$1"

if [ -z "$CSV" ]; then
    echo "Usage: ./run_analysis.sh <csv_file>"
    exit 1
fi

echo "Analyzing: $CSV"
echo "1. Generating Bode plot..."
$PYTHON ../examples/plot_bode.py $CSV

echo "2. Estimating inertia..."
$PYTHON ../examples/estimate_inertia.py $CSV

echo "3. Fitting first-order model..."
$PYTHON ../examples/estimate_damping.py $CSV

echo "4. Comparing model orders..."
$PYTHON ../examples/compare_models.py $CSV

echo ""
echo "✅ Analysis complete!"
echo "Output files:"
ls -lh ${CSV%.*}_*.png
```

**Usage:**
```bash
chmod +x run_analysis.sh
./run_analysis.sh motor_log_aligned_20260608_170004.csv
```

---

## File Structure

```
RobStride_Control_update/
├── cpp/
│   ├── examples/
│   │   ├── plot_bode.py              # Generate Bode plot
│   │   ├── estimate_inertia.py       # Calculate inertia (3 methods)
│   │   ├── estimate_damping.py       # Fit first-order model
│   │   ├── compare_models.py         # Compare model orders
│   │   └── bode.md                   # This file
│   │
│   └── build/
│       ├── multi_motor_functionality # C++ test binary
│       ├── motor_log_aligned_*.csv   # Output CSV files
│       ├── motor_log_aligned_*_bode.png
│       ├── motor_log_aligned_*_fit.png
│       └── motor_log_aligned_*_model_comparison.png
```

---

## Troubleshooting

### "ModuleNotFoundError: scipy"
Install required packages:
```bash
source /home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/venv/bin/activate
pip install scipy pandas matplotlib numpy
```

### CSV file not found
Ensure CSV is in the same directory as the script, or provide full path:
```bash
$PYTHON ../examples/plot_bode.py /full/path/to/motor_log_aligned_20260608_170004.csv
```

### Permission denied for multi_motor_functionality
Use sudo:
```bash
sudo ./multi_motor_functionality --hz 100 --can can0 t 5.0 11 rs04 10
```

### No plots generated
Check that matplotlib backend is configured. Add to script if needed:
```python
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
```

---

## Theory Reference

### Transfer Function (Velocity Output)
$$G(s) = \frac{V(s)}{T(s)} = \frac{1}{I s + B}$$

Where:
- **V(s)** = velocity in rad/s
- **T(s)** = torque in Nm
- **I** = inertia in kg⋅m²
- **B** = damping in Nm⋅s/rad

### Frequency Response
$$G(j\omega) = \frac{1}{B + jI\omega}$$

**Magnitude:**
$$|G(j\omega)| = \frac{1}{\sqrt{B^2 + (I\omega)^2}}$$

**Phase:**
$$\angle G(j\omega) = -\arctan\left(\frac{I\omega}{B}\right)$$

### Key Frequencies
- **Corner frequency:** $\omega_c = B/I$ Hz
  - Below: friction-dominated
  - Above: inertia-dominated

---

## Next Steps

1. **Test multiple motors** to build a parameter database
2. **Use fitted parameters (I, B)** for controller tuning (PID, LQR, etc.)
3. **Compare motors** to identify best performers
4. **Monitor degradation** over time by re-testing periodically

