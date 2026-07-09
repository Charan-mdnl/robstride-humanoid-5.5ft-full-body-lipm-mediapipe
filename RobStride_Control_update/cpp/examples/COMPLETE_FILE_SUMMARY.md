# Complete File Summary

Created a complete **BAM-inspired sinusoidal actuator testing framework** for RobStride motors. All files are in `cpp/examples/`.

## Main Files

**`sinusoidal_actuator_test.cpp`**  
C++ test runner with sinusoidal trajectory generation, CAN control, and CSV logging.

**`plot_sinusoidal_test.py`**  
Python visualizer and metrics tool for generated CSV logs.

**`build_sinusoidal_test.sh`**  
Builds the C++ executable.

**`example_sinusoidal_workflow.sh`**  
Runs three tests and generates plots plus a summary report.

**`requirements_sinusoidal.txt`**  
Python dependencies.

## Quick Start

```bash
pip3 install -r requirements_sinusoidal.txt
bash build_sinusoidal_test.sh
sudo ./sinusoidal_actuator_test 11 simple_sin
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_*.csv --show
```

## Workflow

```bash
bash example_sinusoidal_workflow.sh 11
# Generates full report in: sinusoidal_test_results_YYYYMMDD_HHMMSS/
```

## File Tree

```text
cpp/examples/
├── sinusoidal_actuator_test.cpp        (C++ test runner)
├── plot_sinusoidal_test.py             (Python visualizer)
├── build_sinusoidal_test.sh            (Build script)
├── example_sinusoidal_workflow.sh      (Workflow automation)
├── requirements_sinusoidal.txt         (Python deps)
├── sinusoidal_actuator_test            (Compiled binary)
├── sinusoidal_test_*.csv               (Data files)
└── sinusoidal_plots/
    └── *.png                           (Plot images)
```

## Examples

```bash
sudo ./sinusoidal_actuator_test 11 simple_sin 10 2 1.0 30
python3 plot_sinusoidal_test.py sinusoidal_test_11_simple_sin_*.csv --show
```

```bash
sudo ./sinusoidal_actuator_test 11 sin_time_square 10 2 2.0 45
```

```bash
for id in 11 12 13; do
    sudo ./sinusoidal_actuator_test $id simple_sin 10 2 1.0 30
done
```

## Next Steps

1. **Build:** `bash cpp/examples/build_sinusoidal_test.sh`
2. **Test:** `sudo ./cpp/examples/sinusoidal_actuator_test 11 simple_sin`
3. **Visualize:** `python3 cpp/examples/plot_sinusoidal_test.py <logfile.csv> --show`
4. **Automate:** `bash cpp/examples/example_sinusoidal_workflow.sh 11`
