# RobStride C++ Examples

This folder contains C++ example applications for the RobStride motor control stack. The two main programs are:

- `multi_motor_functionality.cpp` — a real-time command runner for motor zeroing, read-only logging, move+read, torque control, and angle sequences
- `motor_gui.cpp` — a Dear ImGui front-end for building and launching `multi_motor_functionality` commands

## Overview

The C++ examples demonstrate:

- high-frequency MIT-mode control loops using SocketCAN
- non-blocking CSV logging and realtime plotting
- configurable loop frequency and overrun handling
- motor command sequencing with hold time, repeat cycles, and per-step gains
- GUI-driven command generation for convenience

## Build Instructions

From the repository root:

```bash
cd cpp
mkdir -p build
cd build
cmake ..
cmake --build . --target multi_motor_functionality -j4
cmake --build . --target motor_gui -j4
```

If you already have a top-level `make` or `cmake` workflow, you can also build from `cpp/` directly.

### Dependencies

- Linux with SocketCAN support
- C++17 compiler (GCC/Clang)
- CMake 3.12+
- `can-utils` for CAN setup
- `gnuplot` for realtime plotting (optional, the logger still writes CSV without it)
- SDL2, OpenGL, Dear ImGui for `motor_gui`

## `multi_motor_functionality` Command Line Usage

### General CLI format

```bash
sudo ./multi_motor_functionality --hz 100 --r-sec 10 --w-sec 5 --t-sec 5 --overrun warn <cmd> ...
```

### Flags

- `--hz N`
  - sets the control loop frequency in Hertz
  - controls the loop period with `loop_period_ms = 1000.0 / frequency_hz`
  - applies to all modes
- `--r-sec N`
  - duration in seconds for passive read-only mode (`r`)
- `--w-sec N`
  - duration in seconds for move+read mode (`w`)
- `--t-sec N`
  - duration in seconds for torque mode (`t`)
- `--overrun {warn,catchup,skip}`
  - `warn`: print a warning when loop iteration is late
  - `catchup`: advance next scheduled time until the loop is back on schedule
  - `skip`: reschedule from now, effectively dropping missed cycles

> Note: `Frequency` is the loop timing parameter. The `run` fields are mode durations, not loop interval settings.

## Supported Commands

### `z` — Zero all listed motors

```
sudo ./multi_motor_functionality --hz 100 z 0 1.0 0.5 11 rs03 12 rs02
```

This sends zeroing commands to every motor in the list.

### `zp` — Zero a single motor by ID

```
sudo ./multi_motor_functionality --hz 100 zp 11 1.0 0.5 11 rs03 12 rs02
```

This uses the `target_deg` argument slot as the motor ID to zero, while still listing all motors in the call.

### `r` — Passive read-all (zero torque)

```
sudo ./multi_motor_functionality --hz 100 --r-sec 10 r 0 1.0 0.5 11 rs03 12 rs02
```

- reads motor state repeatedly
- writes `motor_log.csv`
- plots frequency if `gnuplot` is installed

### `w` — Move + read all

```
sudo ./multi_motor_functionality --hz 100 --w-sec 10 w 90 1.0 0.5 11 rs03 12 rs02
```

- commands target position for each motor
- uses the same `kp`/`kd` values for all listed motors unless per-motor values are provided
- holds the move in a timed loop while logging state

### `t` — Pure torque control

```
sudo ./multi_motor_functionality --hz 100 --t-sec 10 t 0.0 1.0 0.5 11 rs03 12 rs02
```

- sends pure torque frame with zero position/gains
- reads back motor status while applying torque

### `s` — Angle sequence mode

This is the most advanced mode and supports step-by-step targets and hold behavior.

```bash
sudo ./multi_motor_functionality --hz 100 s <hold_ms> <cycles> <step_count> \
  <angle1> <kp1> <kd1> <torque1> <velocity1> \
  <angle2> <kp2> <kd2> <torque2> <velocity2> ... 11 rs03 12 rs02
```

Example:

```bash
sudo ./multi_motor_functionality --hz 100 s 1000 2 2 \
  0 1.0 0.5 0.0 0.0 \
  90 1.2 0.6 0.0 20.0 \
  11 rs03
```

## Angle Sequence Parameters

The sequence command is parsed as follows:

- `hold_ms`
  - how long to hold each target step, in milliseconds
  - the loop stays in the step until `now >= hold_end`
- `cycles`
  - repeat the full sequence this many times
- `step_count`
  - number of steps defined after the cycle count

Each step is defined by five values:

1. `angle_deg` — target angle in degrees
2. `kp` — proportional gain for this step
3. `kd` — derivative gain for this step
4. `torque_nm` — feedforward torque for this step
5. `velocity_deg_s` — commanded velocity in degrees/sec

If per-step controls are supplied, the loop uses `step.kp`, `step.kd`, `step.torque_ff`, and `step.velocity_deg_s` for that step. Otherwise it falls back to the motor defaults.

## Control Loop Behavior

The loop in `multi_motor_functionality` uses a fixed-frequency schedule:

1. record `loop_start`
2. perform the current mode action (`r`, `w`, `t`, or `s`)
3. collect motor status
4. enqueue log entry and optional console output
5. sleep until `next_time`

This means:

- `Frequency` controls the interval of every loop iteration
- `r_sec`, `w_sec`, and `t_sec` control how long the mode runs for
- sequence mode uses `hold_ms` and `cycles` instead of `run` duration

## Real-Time Logging and Plotting

`multi_motor_functionality` uses `LoggerThread` to decouple logging from the real-time loop.

### CSV output

Logs are written to `motor_log.csv` with columns:

- `timestamp_ms`
- `motor_id`
- `raw`
- `target_deg`
- `target_vel_deg_s`
- `actual_deg`
- `error_deg`
- `vel_deg_s`
- `kp`
- `kd`
- `torque_ff`
- `actual_torque`
- `loop_dt_us`
- `observed_hz`

### Observed frequency

`observed_hz` is computed from the elapsed time between loop starts, not just the body execution time.
This gives a more accurate measurement of the true loop period.

## `motor_gui` Usage

`motor_gui.cpp` provides a graphical command builder for `multi_motor_functionality`.

The GUI lets you:

- add/remove motors and choose motor IDs
- select motor type (`rs00`..`rs04`)
- choose a command: `z`, `zp`, `r`, `w`, `t`, or `s`
- set `Frequency`, `Read run`, `Move run`, `Torque run`, and `Overrun`
- configure angle sequence steps with `hold_ms` and `cycles`
- build and launch the final command without typing it manually

### Example GUI command

The GUI constructs commands like:

```bash
./multi_motor_functionality --hz 600 --r-sec 10 --w-sec 10 --t-sec 10 --overrun warn w 90 1.0 0.5 11 rs03 12 rs02
```

For sequence mode, the GUI emits:

```bash
./multi_motor_functionality --hz 600 --r-sec 10 --w-sec 10 --t-sec 10 --overrun warn s 1000 2 3 0 1.0 0.5 0.0 0.0 45 1.0 0.5 0.0 20.0 90 1.2 0.6 0.0 10.0 11 rs03
```

## Example Workflows

### 1. Fast read-only diagnostic

```bash
sudo ./multi_motor_functionality --hz 600 --r-sec 5 --overrun warn r 0 1.0 0.5 11 rs03
```

### 2. Move to 90 degrees while logging

```bash
sudo ./multi_motor_functionality --hz 300 --w-sec 10 w 90 1.0 0.5 11 rs03
```

### 3. Sequence through angles

```bash
sudo ./multi_motor_functionality --hz 200 s 500 3 3 \
  0 1.0 0.5 0.0 0.0 \
  30 1.0 0.5 0.0 0.0 \
  60 1.0 0.5 0.0 0.0 \
  11 rs03
```

## Tips

- If you want a single runtime for all modes, use the same value for `--r-sec`, `--w-sec`, and `--t-sec`.
- The `Frequency` slider / `--hz` flag is the true control parameter for loop timing.
- `hold_ms` and `cycles` are only used by the `s` angle sequence mode.
- Use `--overrun catchup` when you want the loop to maintain phase alignment despite occasional delays.

## Troubleshooting

- If plotting does not appear, install `gnuplot`:

```bash
sudo apt-get install gnuplot
```

- If the loop drops below frequency, check for blocking I/O or `gnuplot` redraws.
- If sequence timing is wrong, verify `hold_ms` and `cycles` values.

## File Summary

- `multi_motor_functionality.cpp` — main CLI control example with real-time loop, logging, plotting, and sequence support
- `motor_gui.cpp` — GUI builder and launcher for the CLI example

## License

This example follows the repository MIT license.
