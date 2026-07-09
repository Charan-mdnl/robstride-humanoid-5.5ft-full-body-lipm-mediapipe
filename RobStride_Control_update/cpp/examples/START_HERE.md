# 🎯 Sinusoidal Actuator Model Test - START HERE

## What You Got

A **complete BAM-inspired sinusoidal testing framework** for RobStride motor actuators with CAN integration.

### ✅ Includes

- **C++ Test Engine** — Sinusoidal trajectory execution with real-time CAN control
- **Python Plotting** — Publication-quality visualization with automatic metrics  
- **Build Automation** — One-command compilation and workflow execution
- **Complete Documentation** — From 5-minute quickstart to advanced customization
- **Ready-to-Use** — No additional configuration needed (besides CAN setup)

---

## 🚀 Quick Start (3 Steps)

### 1️⃣ Install Python Dependencies
```bash
pip3 install -r requirements_sinusoidal.txt
```

### 2️⃣ Build the Test Binary
```bash
bash build_sinusoidal_test.sh
```

### 3️⃣ Run Your First Test
```bash
sudo ./sinusoidal_actuator_test 11 simple_sin
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_*.csv --show
```

💡 **Replace `11` with your motor's CAN ID**

---

## 📚 Documentation Guide

| Document | Purpose | Time | Best For |
|----------|---------|------|----------|
| **This File** | Overview | 2 min | Getting oriented |
| **QUICKSTART.md** | Setup & basics | 5 min | First-time users |
| **INTEGRATION_GUIDE.md** | Full integration | 20 min | Understanding system |
| **SINUSOIDAL_TEST_README.md** | Complete reference | 30 min | Deep dive & reference |
| **COMPLETE_FILE_SUMMARY.md** | File index | 5 min | Finding things |

---

## 📦 What's Included

### Core Files
```
sinusoidal_actuator_test.cpp     ← C++ test runner (600+ lines)
plot_sinusoidal_test.py           ← Python visualizer (280+ lines)
```

### Build & Execution
```
build_sinusoidal_test.sh          ← One-command build
example_sinusoidal_workflow.sh    ← Automated 3-test workflow
```

### Documentation (All Your Questions Answered)
```
QUICKSTART.md                     ← 5-min setup guide
INTEGRATION_GUIDE.md              ← Integration & customization
SINUSOIDAL_TEST_README.md         ← Complete API reference
COMPLETE_FILE_SUMMARY.md          ← File index & overview
requirements_sinusoidal.txt       ← Python dependencies
```

---

## 🎮 Test Types Available

```bash
# Single frequency (best for tuning)
sudo ./sinusoidal_actuator_test 11 simple_sin

# Multi-frequency (test harmonic response)
sudo ./sinusoidal_actuator_test 11 sin_sin

# Progressive sweep (characterize across frequencies)
sudo ./sinusoidal_actuator_test 11 sin_time_square
```

---

## 📊 Output: 3 Professional Plots

1. **Position Tracking** — How well motor follows target
2. **Error & Torque** — Tracking error and motor output
3. **Phase & Power** — Advanced analysis

Plus automatic metrics:
- Mean Absolute Error (MAE)
- RMS Error
- Max error
- Torque range
- Power consumption

---

## 🔄 Complete Workflow (One Command)

Runs 3 different tests automatically, generates all plots and reports:

```bash
bash example_sinusoidal_workflow.sh 11
```

Creates organized output:
```
sinusoidal_test_results_YYYYMMDD_HHMMSS/
├── SUMMARY.txt              ← Comprehensive report
├── 3x CSV data files
└── plots/
    └── 9x PNG plot files
```
