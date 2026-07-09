# RobStride Arduino Control

Arduino平台实现RobStride电机控制库，支持ESP32和Arduino系列微控制器。

## 支持的开发板

- **ESP32**: 推荐使用，内置TWAI/CAN控制器
- **Arduino Mega 2560**: 需要外部CAN模块(MCP2515)
- **Arduino Uno**: 需要外部CAN模块(MCP2515)

## 项目结构

```
arduino/
├── README.md                    # Arduino文档
├── simple_joint_control/        # 简单关节控制
│   ├── simple_joint_control.ino # 主程序
│   ├── TWAI_CAN_MI_Motor.h      # CAN驱动头文件
│   └── TWAI_CAN_MI_Motor.cpp    # CAN驱动实现
├── joint_position_control/       # 位置控制
├── dual_motor_control/          # 双电机控制
├── advanced_motor_control/      # 高级控制
├── mi_motor_control/           # 基础控制
└── sim_setup_motor/            # 电机设置
```

## 硬件连接

### ESP32直接连接
```
ESP32           CAN收发器           RobStride电机
-----           ----------           ------------
GPIO5    <----> TX       <----> CAN_H
GPIO4    <----> RX       <----> CAN_L
3.3V     -----> VCC
GND      -----> GND
```

### Arduino + MCP2515模块
```
Arduino        MCP2515            CAN收发器           RobStride电机
--------        --------            ----------           ------------
D10(SPI_SS)   CS
D11(SPI_MOSI)  SI/MOSI
D12(SPI_MISO)  SO/MISO
D13(SPI_SCK)   SCK/SCLK
5V             VCC
GND            GND
MCP2515 TX   -> CAN_H -> 电机
MCP2515 RX   -> CAN_L -> 电机
```

## 快速开始

### 1. 环境配置

#### Arduino IDE
1. 安装Arduino IDE 1.8.19+
2. 安装ESP32开发板支持
3. 工具 -> 开发板 -> ESP32 Arduino -> ESP32 Dev Module

#### PlatformIO
```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
```

### 2. 编译上传

```bash
# 使用Arduino IDE
1. 打开对应的.ino文件
2. 选择正确的开发板
3. 上传程序

# 使用PlatformIO
pio run --target upload
```

## 控制示例

### 基础位置控制

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

### 参数调整

```cpp
// 位置增益 (0-500.0)
motor.set_kp(30.0);

// 阻尼增益 (0-100.0)
motor.set_kd(0.5);

// 速度限制 (rad/s)
motor.set_velocity_limit(20.0);

// 扭矩限制 (Nm)
motor.set_torque_limit(30.0);
```

## 控制模式

### MIT模式 (Mode 0)
直接控制位置、速度和扭矩。

```cpp
// MIT控制指令
void send_mit_command(float position, float kp, float kd);
```

### 位置模式 (Mode 1)
基于内部PID的位置控制。

```cpp
// 设置位置目标
void set_position_target(float position);

// 设置PID参数
void set_position_kp(float kp);
void set_position_kd(float kd);
```

### 速度模式 (Mode 2)
速度闭环控制。

```cpp
// 设置速度目标
void set_velocity_target(float velocity);

// 设置速度PID
void set_velocity_kp(float kp);
void set_velocity_ki(float ki);
```

## 通信协议

### CAN帧格式
- **波特率**: 1000kbps
- **数据长度**: 8字节
- **扩展ID**: 29位标准CAN扩展帧

### 命令类型
- **0x00**: 获取设备ID
- **0x01**: 操作控制 (MIT模式)
- **0x02**: 操作状态反馈
- **0x03**: 使能电机
- **0x04**: 禁用电机
- **0x18**: 写入参数

## 项目说明

### simple_joint_control
最简单的关节控制示例，演示基础MIT控制。
- 控制单个电机
- 交互式参数调整
- 实时状态反馈

### joint_position_control
位置控制专用示例。
- 平滑轨迹规划
- 多段位运动
- 位置误差监控

### dual_motor_control
双电机协调控制。
- 同步运动
- 主从控制
- 协调轨迹

### advanced_motor_control
高级控制功能。
- 自适应控制
- 振动抑制
- 温度监控

## 故障排除

### 通信问题
1. **CAN总线错误**
   - 检查波特率设置 (1000kbps)
   - 验证终端电阻 (120Ω)
   - 确认CAN_H/CAN_L接线

2. **电机无响应**
   - 检查电机ID设置
   - 验证电源供应 (12-48V)
   - 确认使能指令发送成功

### 编译问题
1. **ESP32编译错误**
   - 更新ESP32开发板包
   - 检查Arduino IDE版本
   - 清理编译缓存

2. **库依赖问题**
   - 确保包含所有头文件
   - 检查TWAI库版本
   - 验证SPI连接(MCP2515)

## 性能指标

| 开发板 | 控制频率 | 延迟 | 电机数量 | 特点 |
|--------|----------|------|----------|------|
| ESP32  | 100-200Hz | 2-5ms | 1-8 | 内置CAN，高性能 |
| Arduino Mega | 50-100Hz | 5-10ms | 1-4 | 丰富IO，稳定 |
| Arduino Uno | 30-50Hz | 10-20ms | 1-2 | 低成本，入门级 |

## 开发资源

### Arduino库依赖
```cpp
#include <Arduino.h>
#include <driver/twai.h>  // ESP32 TWAI驱动
#include <SPI.h>          // SPI通信(MCP2515)
```

### 调试工具
- **串口监视器**: 115200波特率
- **CAN分析仪**: 实时监控CAN流量
- **逻辑分析仪**: 分析时序信号

## 扩展功能

### 多电机控制
```cpp
TWAI_CAN_MI_Motor motor1(11);
TWAI_CAN_MI_Motor motor2(12);
TWAI_CAN_MI_Motor motor3(13);

void setup() {
    motor1.init(CAN_SPEED_1000KBPS);
    motor2.init(CAN_SPEED_1000KBPS);
    motor3.init(CAN_SPEED_1000KBPS);
}
```

### 传感器集成
```cpp
// 编码器反馈
float encoder_position = read_encoder();

// 力矩传感器
float torque_feedback = read_torque_sensor();
```

### 网络控制
```cpp
// WiFi控制
#include <WiFi.h>
#include <WebServer.h>

// 通过网页控制电机
void handleControlRequest() {
    float target_pos = server.arg("pos").toFloat();
    motor.send_mit_command(target_pos, kp, kd);
}
```

## 许可证

MIT License - 详见[../LICENSE](../LICENSE)文件

## 技术支持

- 项目源码: https://github.com/tianrking/robstride-control
- 问题报告: https://github.com/tianrking/robstride-control/issues