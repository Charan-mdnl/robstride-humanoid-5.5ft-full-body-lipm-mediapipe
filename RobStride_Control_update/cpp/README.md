# RobStride Control C++

C++ 实现 RobStride 电机控制库，提供高性能的实时控制功能。

## 特性

- ⚡ **高性能**：200Hz 控制频率，1ms 延迟
- 🔧 **直接控制**：基于 SocketCAN 的底层实现
- 🛡️ **类型安全**：强类型检查，内存安全
- 📦 **易于集成**：标准 CMake 构建系统
- 🎯 **专业级**：适用于工业级应用

## 系统要求

- Linux 系统 (Ubuntu 18.04+, Debian 10+)
- GCC 7+ 或 Clang 8+
- CMake 3.12+
- SocketCAN 支持

## 安装依赖

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install build-essential cmake can-utils

# 使用项目脚本
make install-deps
```

## 编译

### 使用 Makefile

```bash
# 编译
make

# 调试版本
make debug

# 发布版本
make release

# 安装到系统
sudo make install
```

### 使用 CMake

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)

# 可选：安装
sudo make install
```

## 使用方法

### 基本使用

```cpp
#include "can_interface.h"
#include "protocol.h"

int main(int argc, char* argv[]) {
    int motor_id = 11;

    // 初始化 CAN 接口
    CanInterface can;
    if (!can.init("can0")) {
        std::cerr << "Failed to initialize CAN" << std::endl;
        return 1;
    }

    // 设置电机参数
    enable_motor(can.socket(), motor_id);
    set_mode_raw(can.socket(), motor_id, ControlMode::MIT_MODE);

    // 设置限制
    write_limit(can.socket(), motor_id, ParamID::VELOCITY_LIMIT, 20.0);
    write_limit(can.socket(), motor_id, ParamID::TORQUE_LIMIT, 20.0);

    // 位置控制
    double target_pos = M_PI / 2; // 90度
    write_operation_frame(can.socket(), motor_id, target_pos, 30.0, 0.5);

    return 0;
}
```

### 编译运行

```bash
# 编译
make

# 运行（需要 sudo 权限）
sudo ./build/robstride-mit-position 11
```

## API 参考

### CanInterface 类

```cpp
class CanInterface {
public:
    CanInterface();
    ~CanInterface();

    bool init(const std::string& interface = "can0");
    void close();
    bool send_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
    bool read_frame(can_frame* frame, int timeout_ms = 100);
    bool is_ready() const;
};
```

### 协议函数

```cpp
// 电机控制
bool enable_motor(int socket, int motor_id);
bool set_mode_raw(int socket, int motor_id, int8_t mode);
bool write_operation_frame(int socket, int motor_id,
                          double pos, double kp, double kd);

// 参数设置
bool write_limit(int socket, int motor_id, uint16_t param_id, float limit);

// 状态读取
bool read_operation_frame(int socket);
```

### 协议常量

```cpp
namespace CommType {
    constexpr uint32_t ENABLE = 3;
    constexpr uint32_t OPERATION_CONTROL = 1;
    constexpr uint32_t WRITE_PARAMETER = 18;
}

namespace ControlMode {
    constexpr int8_t MIT_MODE = 0;
    constexpr int8_t POSITION_MODE = 1;
    constexpr int8_t SPEED_MODE = 2;
}
```

## 控制模式

### MIT 模式 (Mode 0)

```cpp
// 切换到 MIT 模式
set_mode_raw(socket, motor_id, ControlMode::MIT_MODE);

// 发送位置指令
double position = M_PI / 2;    // 90度
double kp = 30.0;              // 位置增益
double kd = 0.5;               // 阻尼增益

while (running) {
    write_operation_frame(socket, motor_id, position, kp, kd);
    read_operation_frame(socket);  // 清空接收缓冲区

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
}
```

### 参数设置

```cpp
// 设置控制参数
write_limit(socket, motor_id, ParamID::VELOCITY_LIMIT, 20.0);
write_limit(socket, motor_id, ParamID::TORQUE_LIMIT, 20.0);
write_limit(socket, motor_id, ParamID::POSITION_KP, 30.0);
write_limit(socket, motor_id, ParamID::VELOCITY_KP, 0.5);
```

## 交互式控制

程序启动后提供交互式界面：

```
🎯 MIT 位置控制台 (ID: 11)
========================================
👉 输入数字 (度) 回车即可改变位置
👉 'kp <值>' (例如: kp 100) 来调节刚度
👉 'kd <值>' (例如: kd 2.0) 来调节阻尼 (防抖)
👉 '0' 或 'home' 回到零点
👉 'q' 退出
⚠️  当前 Kp=100 | Kd=2.0
----------------------------------------
[0.0°] >> 90
 -> 目标设定: 90.0°
```

## 性能优化

### 编译优化

```bash
# 发布版本（优化）
CXXFLAGS="-O3 -DNDEBUG" make

# 启用 LTO
cmake -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=TRUE ..
```

### 运行时优化

```cpp
// 高优先级调度
struct sched_param param;
param.sched_priority = 99;
sched_setscheduler(0, SCHED_FIFO, &param);

// CPU 亲和性
cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(2, &cpuset);  // 绑定到 CPU 2
sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
```

### 内存优化

```cpp
// 使用对象池
std::vector<can_frame> frame_pool;
frame_pool.reserve(1000);  // 预分配

// 避免动态分配
uint8_t data[8];  // 栈分配
```

## 调试

### CAN 监控

```bash
# 监控 CAN 流量
sudo candump can0

# 过滤特定 ID
sudo candump can0,0C0:7FF
```

### 调试输出

```cpp
// 编译时启用调试
#ifdef DEBUG
    std::cout << "Debug: " << message << std::endl;
#endif

// 运行时调试
const bool debug = true;
if (debug) {
    printf("Pos: %.3f, Kp: %.1f, Kd: %.1f\n", pos, kp, kd);
}
```

## 测试

### 单元测试

```bash
# 安装 Google Test
sudo apt-get install libgtest-dev

# 编译测试
cmake -DBUILD_TESTING=ON ..
make

# 运行测试
./tests/robstride_test
```

### 集成测试

```bash
# 电机连接测试
make test

# 手动测试
sudo ./build/robstride-mit-position --test

# 运行示例程序
g++ -std=c++17 -I../include examples/basic_control.cpp -o basic_control
sudo ./basic_control 11
```

## 故障排除

### 编译错误

```bash
# 检查 C++ 标准
g++ --version  # 需要 GCC 7+

# 检查 CMake
cmake --version  # 需要 3.12+

# 清理重新编译
make clean
make
```

### 运行时错误

```bash
# 检查 CAN 接口
ip link show can0

# 检查权限
groups  # 应该包含 dialout

# 检查设备
ls -l /sys/class/net/can0
```

## 部署

### 系统服务

```bash
# 创建服务文件
sudo cp scripts/robstride.service /etc/systemd/system/
sudo systemctl enable robstride
sudo systemctl start robstride
```

### Docker

```dockerfile
FROM ubuntu:20.04
RUN apt-get update && apt-get install -y build-essential cmake
COPY . /app
WORKDIR /app
RUN make
CMD ["./build/robstride-mit-position"]
```

## 许可证

MIT License - 详见 [LICENSE](../LICENSE) 文件

## 贡献

欢迎提交 Issue 和 Pull Request！

## 支持

- 📖 [完整文档](../docs/)
- 🐛 [问题反馈](https://github.com/tianrking/robstride-control/issues)