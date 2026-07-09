# RobStride Control

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B17)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green.svg)](https://www.python.org/downloads/)
[![Rust](https://img.shields.io/badge/Rust-1.70%2B-orange.svg)](https://www.rust-lang.org/)
[![Arduino](https://img.shields.io/badge/Arduino-ESP32-00979D.svg)](https://www.arduino.cc/)

High-performance motor control library for RobStride motors with implementations in Python, C++, Rust, and Arduino.

## Quick Start

```bash
# Environment setup
sudo apt-get install can-utils
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Python
cd python && pip install -r requirements.txt
python3 src/position_control.py 11

# C++
cd cpp && make && sudo ./build/robstride-mit-position 11

# Rust
cd rust && cargo run --release -- 11

# Arduino
# Open Arduino IDE, load arduino/simple_joint_control/simple_joint_control.ino
# Select ESP32 Dev Module and upload
```

## Languages

| Language | Control Frequency | Latency | Memory Usage | Platform |
|----------|------------------|--------|-------------|---------|
| Python   | 100 Hz  | 5ms  | 50MB     | Linux |
| C++      | 200 Hz  | 1ms  | 10MB     | Linux |
| Rust     | 150 Hz  | 2ms  | 15MB     | Linux |
| Arduino  | 50-200Hz| 2-20ms| 10-50KB  | ESP32/MCU |

## Project Structure

```
├── python/          # Python implementation
├── cpp/             # C++ implementation
├── rust/            # Rust implementation
├── arduino/         # Arduino implementation
└── scripts/         # Build and setup scripts
```

## Supported Motors

- **RS-00**: 17 Nm, 50 rad/s
- **RS-01**: 17 Nm, 44 rad/s
- **RS-02**: 17 Nm, 44 rad/s
- **RS-03**: 60 Nm, 50 rad/s
- **RS-04**: 120 Nm, 15 rad/s
- **RS-05**: 17 Nm, 33 rad/s
- **RS-06**: 60 Nm, 20 rad/s

## Documentation

- [中文文档](README_zh.md)
- [Español](README_es.md)
- [Python](python/README.md)
- [C++](cpp/README.md)
- [Rust](rust/README.md)
- [Arduino](arduino/README.md)

## Examples

### Python
```python
from src.position_control import PositionControllerMIT
controller = PositionControllerMIT(11)
controller.connect()
controller.set_angle(90.0)
```

### C++
```cpp
CanInterface can;
can.init("can0");
enable_motor(can.socket(), 11);
write_operation_frame(can.socket(), 11, M_PI/2, 30.0, 0.5);
```

### Rust
```rust
let socket = Arc::new(Mutex::new(CanSocket::open("can0")?));
enable_motor(&socket, 11)?;
write_operation_frame(&socket.lock()?, 11, std::f64::consts::PI/2.0, 30.0, 0.5)?;
```

### Arduino
```cpp
TWAI_CAN_MI_Motor motor(11);
motor.init(CAN_SPEED_1000KBPS);
motor.enable_motor();
motor.send_mit_command(PI/2, 30.0, 0.5);
```

## License

MIT License - see [LICENSE](LICENSE) file

## Support

- Issues: https://github.com/tianrking/robstride-control/issues