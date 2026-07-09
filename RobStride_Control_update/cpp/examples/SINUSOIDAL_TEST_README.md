# Sinusoidal Actuator Test

Inspired by [Rhoban BAM](https://github.com/Rhoban/bam), this test framework characterizes servo actuator behavior under sinusoidal trajectories. It's designed to understand friction, backlash, and control performance across varying frequencies and amplitudes.

## Files

1. **sinusoidal_actuator_test.cpp** — C++ test runner with CAN integration
2. **plot_sinusoidal_test.py** — Python visualization and analysis tool
3. **build_sinusoidal_test.sh** — Build helper
4. **example_sinusoidal_workflow.sh** — Automated multi-test workflow
5. **requirements_sinusoidal.txt** — Python dependencies

## Build

```bash
bash build_sinusoidal_test.sh
```

## Running Tests

```bash
sudo ./sinusoidal_actuator_test 11 sin_time_square
sudo ./sinusoidal_actuator_test 11 sin_sin 8 1.5 1.5 30
sudo ./sinusoidal_actuator_test 11 simple_sin 12 3 2.0 45
```

## Plotting Results

```bash
python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_time_square_20250520_120000.csv --show
python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_time_square_20250520_120000.csv --save
python3 plot_sinusoidal_test.py sinusoidal_test_11_sin_time_square_20250520_120000.csv --show --save
```

## Command Reference

```bash
sinusoidal_actuator_test <motor_id> <trajectory_type> [kp] [kd] [frequency_hz] [amplitude_deg]
```

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `motor_id` | int | required | 1-255 | CAN motor ID |
| `trajectory_type` | string | required | sin_time_square, sin_sin, simple_sin | Trajectory pattern |
| `kp` | float | 10.0 | 0-100 | Proportional gain |
| `kd` | float | 2.0 | 0-50 | Derivative gain |
| `frequency_hz` | float | 1.0 | 0.1-10 | Fundamental frequency |
| `amplitude_deg` | float | 30.0 | 1-180 | Peak amplitude in degrees |

### Plot Script

```bash
python3 plot_sinusoidal_test.py <logfile> [--show] [--save] [--output DIR]
```

| Flag | Effect |
|------|--------|
| `--show` | Display plots in interactive window |
| `--save` | Save plots as PNG files to disk |
| `--output DIR` | Specify output directory (default: `./sinusoidal_plots`) |

## Data Format

CSV columns:

| Column | Unit | Description |
|--------|------|-------------|
| `time_s` | seconds | Elapsed time |
| `target_pos_deg` | degrees | Desired position |
| `target_vel_deg_s` | deg/s | Desired velocity |
| `actual_pos_deg` | degrees | Motor position feedback |
| `actual_vel_deg_s` | deg/s | Motor velocity feedback |
| `actual_torque_nm` | N·m | Motor torque output |
| `kp` | - | Proportional gain |
| `kd` | - | Derivative gain |
| `error_deg` | degrees | Position error |
| `power_w` | Watts | Instantaneous power |

## Example Workflow

```bash
sudo ./sinusoidal_actuator_test 11 simple_sin 10 2 1.0 30
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_20250520_120000.csv --show
```

```bash
bash example_sinusoidal_workflow.sh 11
```
