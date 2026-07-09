# Integration Guide

You now have a complete **BAM-inspired sinusoidal testing framework** for characterizing RobStride motor actuators.

## Files

| File | Purpose | Language |
|------|---------|----------|
| `sinusoidal_actuator_test.cpp` | Test executor with trajectory generation | C++ |
| `plot_sinusoidal_test.py` | Data analysis and visualization | Python |
| `build_sinusoidal_test.sh` | One-command build script | Bash |
| `example_sinusoidal_workflow.sh` | Multi-test workflow example | Bash |
| `requirements_sinusoidal.txt` | Python dependencies | Text |

## Setup

```bash
pip3 install -r cpp/examples/requirements_sinusoidal.txt
cd cpp/examples
bash build_sinusoidal_test.sh
```

## First Test

```bash
sudo ./sinusoidal_actuator_test 11 simple_sin
```

The test saves a CSV file like:

```text
sinusoidal_test_11_simple_sin_20250520_160000.csv
```

Visualize it with:

```bash
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_20250520_160000.csv --show
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_20250520_160000.csv --save
```

## Common Experiments

```bash
sudo ./sinusoidal_actuator_test 11 simple_sin 5 1 1.0 30
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_*.csv --show

sudo ./sinusoidal_actuator_test 11 simple_sin 10 2 1.0 30
sudo ./sinusoidal_actuator_test 11 simple_sin 15 3 1.0 30
```

Frequency sweep:

```bash
sudo ./sinusoidal_actuator_test 11 sin_time_square 10 2 2.0 45
python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_time_square_*.csv --show
```

Multi-motor examples:

```bash
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_*.csv --save
python3 plot_sinusoidal_test.py sinusoidal_test_12_simple_sin_*.csv --save
python3 plot_sinusoidal_test.py sinusoidal_test_13_simple_sin_*.csv --save
```

## Workflow

```bash
bash example_sinusoidal_workflow.sh 11
```

Creates:

```text
sinusoidal_test_results_YYYYMMDD_HHMMSS/
```

## Troubleshooting

- CAN interface failed: check `ip link show can0`
- Motor does not move: verify motor ID and power supply
- Plotting issues: install dependencies with `pip3 install -r requirements_sinusoidal.txt`

**Visualize:** `python3 cpp/examples/plot_sinusoidal_test.py <logfile.csv> --show`
