# RobStride Control

RobStride电机控制库，提供Python、C++、Rust、Arduino四种语言的电机控制实现。

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B17)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green.svg)](https://www.python.org/downloads/)
[![Rust](https://img.shields.io/badge/Rust-1.70%2B-orange.svg)](https://www.rust-lang.org/)
[![Arduino](https://img.shields.io/badge/Arduino-ESP32-00979D.svg)](https://www.arduino.cc/)

## 项目特性

- **多语言支持**：Python（易用性）、C++（高性能）、Rust（安全性）、Arduino（嵌入式）
- **多种控制模式**：MIT模式、位置控制、速度控制、扭矩控制
- **实时性能**：高频率控制循环，低延迟响应
- **工业级可靠性**：完善的错误处理和安全保护机制
- **跨平台支持**：Linux系统、嵌入式控制器

## 支持的电机型号

| 型号 | 最大扭矩 | 最大速度 | KP范围 | KD范围 |
|------|----------|----------|--------|--------|
| RS-00 | 17 Nm | 50 rad/s | 500.0 | 5.0 |
| RS-01 | 17 Nm | 44 rad/s | 500.0 | 5.0 |
| RS-02 | 17 Nm | 44 rad/s | 500.0 | 5.0 |
| RS-03 | 60 Nm | 50 rad/s | 5000.0| 100.0|
| RS-04 | 120 Nm| 15 rad/s | 5000.0| 100.0|
| RS-05 | 17 Nm | 33 rad/s | 500.0 | 5.0 |
| RS-06 | 60 Nm | 20 rad/s | 5000.0| 100.0|

## 项目结构

```
RobStride-Control/
├── python/              # Python实现
│   ├── src/            # 源代码
│   ├── examples/       # 示例程序
│   └── robstride_dynamics/  # Python SDK
├── cpp/                 # C++实现
│   ├── src/            # 源代码
│   ├── include/        # 头文件
│   └── examples/       # 示例程序
├── rust/                # Rust实现
│   ├── src/            # 源代码
│   └── examples/       # 示例程序
├── arduino/             # Arduino实现
│   ├── simple_joint_control/    # 简单关节控制
│   ├── joint_position_control/   # 位置控制
│   ├── dual_motor_control/       # 双电机控制
│   └── advanced_motor_control/   # 高级控制
└── scripts/             # 构建和安装脚本
```

## 快速开始

### 系统要求

- **操作系统**：Linux (Ubuntu 18.04+ / Debian 10+)
- **硬件**：支持SocketCAN的CAN接口
- **权限**：root权限或sudo访问CAN设备

### 环境配置

```bash
# 安装CAN工具
sudo apt-get install can-utils

# 设置CAN接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 运行环境配置脚本
./scripts/setup.sh
```

### Python版本

```bash
cd python
pip install -r requirements.txt

# 位置控制
python3 src/position_control.py 11

# 速度控制
python3 src/speed_control.py 11

# 示例程序
python3 examples/basic_usage.py 11
```

### C++版本

```bash
cd cpp
make install-deps
make

# 运行主程序
sudo ./build/robstride-mit-position 11

# 运行示例
g++ -std=c++17 -I../include examples/basic_control.cpp -o basic_control
sudo ./basic_control 11
```

### Rust版本

```bash
cd rust
cargo build --release

# 运行主程序
cargo run --release --bin robstride-mit-position -- 11

# 运行示例
cargo run --release --bin basic_control -- 11
```

### Arduino版本

```bash
cd arduino

# 使用Arduino IDE
1. 打开 simple_joint_control/simple_joint_control.ino
2. 选择 ESP32 Dev Module
3. 上传程序

# 使用PlatformIO
pio run --target upload

# 不同控制示例
- simple_joint_control     # 基础关节控制
- joint_position_control    # 位置控制
- dual_motor_control        # 双电机控制
- advanced_motor_control    # 高级控制
```

## 性能指标

| 实现语言 | 控制频率 | 延迟 | CPU使用率 | 内存占用 | 平台 |
|----------|----------|------|-----------|----------|------|
| Python   | 100 Hz   | 5ms  | 15%       | 50MB     | Linux |
| C++      | 200 Hz   | 1ms  | 5%        | 10MB     | Linux |
| Rust     | 150 Hz   | 2ms  | 8%        | 15MB     | Linux |
| Arduino  | 50-200Hz | 2-20ms| 5-15%     | 10-50KB  | ESP32/MCU |

## 控制模式

### MIT模式 (Mode 0)
直接发送位置、速度、扭矩目标，适用于高性能控制应用。
- **控制频率**：50-100Hz
- **应用场景**：机器人关节控制、精密定位

### 位置模式 (Mode 1)
基于内部位置环的位置控制。
- **控制频率**：20-50Hz
- **应用场景**：点到点运动，轨迹跟踪

### 速度模式 (Mode 2)
速度闭环控制。
- **控制频率**：20-50Hz
- **应用场景**：传送带控制，轮式机器人

## 使用示例

### Python MIT位置控制

```python
from src.position_control import PositionControllerMIT

# 初始化控制器
controller = PositionControllerMIT(motor_id=11)
controller.connect()

# 设置位置到90度
controller.set_angle(90.0)

# 调整控制参数
controller.set_kp(30.0)  # 位置增益
controller.set_kd(0.5)   # 阻尼增益
```

### C++ MIT位置控制

```cpp
#include "can_interface.h"
#include "protocol.h"

CanInterface can;
can.init("can0");

// 设置电机参数
enable_motor(can.socket(), 11);
set_mode_raw(can.socket(), 11, ControlMode::MIT_MODE);

// 发送位置指令
write_operation_frame(can.socket(), 11, M_PI/2, 30.0, 0.5);
```

### Rust MIT位置控制

```rust
let socket = Arc::new(Mutex::new(CanSocket::open("can0")?));

// 启用电机并设置模式
enable_motor(&socket, 11)?;
set_mode_raw(&socket, 11, 0)?;

// 发送位置指令
write_operation_frame(&socket.lock()?, 11, std::f64::consts::PI/2.0, 30.0, 0.5)?;
```

### Arduino MIT位置控制

```cpp
#include "TWAI_CAN_MI_Motor.h"

TWAI_CAN_MI_Motor motor(11);  // 电机ID=11

void setup() {
    Serial.begin(115200);
    motor.init(CAN_SPEED_1000KBPS);
    motor.enable_motor();
}

void loop() {
    // 设置位置到90度
    motor.send_mit_command(PI/2, 30.0, 0.5);
    delay(2000);

    // 回到原点
    motor.send_mit_command(0, 30.0, 0.5);
    delay(2000);
}
```

## 交互式控制

### 位置控制命令
- `90` - 设置到90度
- `-45` - 设置到-45度
- `kp 30` - 设置位置增益为30
- `kd 0.5` - 设置阻尼增益为0.5
- `home` - 回零位
- `status` - 显示状态
- `quit` - 退出

### 速度控制命令
- `5.0` - 设置正向5 rad/s
- `-3.0` - 设置反向3 rad/s
- `stop` - 停止电机
- `status` - 显示状态
- `quit` - 退出

## 故障排除

### 常见问题

1. **找不到电机**
   ```bash
   # 检查CAN连接
   sudo ip link show can0
   # 扫描电机
   python3 -c "from robstride_dynamics import RobstrideBus; print(RobstrideBus.scan_channel('can0'))"
   ```

2. **权限被拒绝**
   ```bash
   # 添加用户到dialout组
   sudo usermod -a -G dialout $USER
   # 重启系统
   ```

3. **控制不稳定**
   - 调整Kp/Kd参数
   - 检查CAN总线负载
   - 验证电源稳定性

### 调试工具

```bash
# 监控CAN流量
sudo candump can0

# 过滤特定ID
sudo candump can0,0C0:7FF

# 检查CAN统计
sudo ip -details link show can0
```

## 开发指南

### 编译要求

- **Python**: Python 3.8+, python-can
- **C++**: GCC 7+或Clang 8+, CMake 3.12+
- **Rust**: Rust 1.70+, Cargo
- **Arduino**: Arduino IDE 1.8.19+, ESP32支持

### 测试

```bash
# Python测试
python3 examples/basic_usage.py 11

# C++测试
make test

# Rust测试
cargo test

# Arduino测试
通过Arduino IDE串口监视器查看调试信息
```

### 贡献代码

1. Fork项目
2. 创建功能分支
3. 提交更改
4. 创建Pull Request

## 许可证

MIT License - 详见[LICENSE](LICENSE)文件

## 技术支持

- Issues: https://github.com/tianrking/robstride-control/issues