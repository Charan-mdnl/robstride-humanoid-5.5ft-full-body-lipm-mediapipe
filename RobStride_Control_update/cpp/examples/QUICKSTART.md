# Quick Start - Sinusoidal Actuator Test

## 5-Minute Setup

### 1. Install Python Dependencies
```bash
pip3 install -r requirements_sinusoidal.txt
```

### 2. Build the Test Binary
```bash
bash build_sinusoidal_test.sh
```

### 3. Run Your First Test
```bash
# Replace 11 with your motor's CAN ID
sudo ./sinusoidal_actuator_test 11 simple_sin
```

### 4. Visualize Results
```bash
# List generated log files
ls -la sinusoidal_test_*.csv

# Plot the most recent test
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin*.csv --show
```

## What You'll See

1. **Real-time Console Output:**
   ```
   ━━━ SINUSOIDAL ACTUATOR TEST ━━━
   Motor ID:       11
   Trajectory:     simple_sin
   Frequency:      1.0 Hz
   Amplitude:      30 deg
   Duration:       30 s
   ✅ Test completed
   ```

2. **Three Analysis Plots:**
   - **Position Tracking** — Target vs actual position
   - **Error & Torque** — Tracking error and motor torque output
   - **Phase & Power** — Phase plane and power consumption

## Test Scenarios

### Scenario A: Basic Tuning
```bash
# Test with conservative gains
sudo ./sinusoidal_actuator_test 11 simple_sin 5 1 1.0 30
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin*.csv --show

# Then try aggressive gains and compare
sudo ./sinusoidal_actuator_test 11 simple_sin 15 3 1.0 30
```

### Scenario B: Full Characterization (3 tests)
```bash
bash example_sinusoidal_workflow.sh 11
# Outputs: sinusoidal_test_results_YYYYMMDD_HHMMSS/
```

### Scenario C: Frequency Sweep
```bash
# See how the motor handles increasing frequency
sudo ./sinusoidal_actuator_test 11 sin_time_square 10 2 2.0 45
python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_time_square*.csv --show
```

## Common Commands

| Task | Command |
|------|---------|
| Build | `bash build_sinusoidal_test.sh` |
| Test (simple) | `sudo ./sinusoidal_actuator_test 11 simple_sin` |
| Test (custom) | `sudo ./sinusoidal_actuator_test 11 sin_sin 8 1.5 1.5 30` |
| Plot (show) | `python3 plot_sinusoidal_test.py file.csv --show` |
| Plot (save) | `python3 plot_sinusoidal_test.py file.csv --save` |
| Batch test | `bash example_sinusoidal_workflow.sh 11` |
| Check CAN | `ip link show can0` |

## File Organization

```
cpp/examples/
├── sinusoidal_actuator_test.cpp          ← Test runner (C++)
├── plot_sinusoidal_test.py               ← Visualization (Python)
├── build_sinusoidal_test.sh              ← Build script
├── example_sinusoidal_workflow.sh        ← Complete workflow example
├── requirements_sinusoidal.txt           ← Python dependencies
├── SINUSOIDAL_TEST_README.md             ← Full documentation
├── QUICKSTART.md                         ← This file
│
└── sinusoidal_actuator_test              ← Compiled binary (after build)
└── sinusoidal_test_*.csv                 ← Generated data files
└── sinusoidal_plots/                     ← Generated plots (PNG files)
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "CAN interface failed" | Check: `ip link show can0` and `ifconfig can0` |
| Motor doesn't move | Verify motor ID and power supply |
| "Plot script not found" | Run from `cpp/examples/` directory |
| Missing Python packages | `pip3 install -r requirements_sinusoidal.txt` |
| No plots display | Use `--save` instead of `--show` |

## Next Steps

- See **SINUSOIDAL_TEST_README.md** for complete documentation
- Check **example_sinusoidal_workflow.sh** for multi-test workflows
- Modify **sinusoidal_actuator_test.cpp** to customize trajectories
- Compare results across different motors to characterize variance

## Integration with CANable

The test automatically integrates with your CAN interface:

1. Initializes CAN (default: `can0`)
2. Enables the motor
3. Sets MIT mode (position + velocity + torque control)
4. Executes trajectory with PD control
5. Logs all data to CSV

No additional configuration needed if your CAN interface is up:
```bash
# Verify CAN is ready
ip link show can0

# If not up, initialize it
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

---

**That's it!** You now have a BAM-inspired sinusoidal testing framework. 🎉
