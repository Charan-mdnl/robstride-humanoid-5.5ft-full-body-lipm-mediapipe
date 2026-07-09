# RobStride Control Rust

Rust 实现 RobStride 电机控制库，提供内存安全和并发安全的电机控制功能。

## 特性

- 🦀 **内存安全**：Rust 的内存安全保证，无需垃圾回收
- ⚡ **高性能**：150Hz 控制频率，2ms 延迟
- 🔒 **并发安全**：Arc + Mutex 确保线程安全
- 🎯 **类型安全**：编译时类型检查
- 📦 **易于使用**：Cargo 生态系统支持

## 系统要求

- Rust 1.70+
- Linux 系统 (SocketCAN 支持)
- CAN 接口硬件

## 安装

### 安装 Rust

```bash
# 安装 Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 重启 shell 或运行
source "$HOME/.cargo/env"
```

### 编译项目

```bash
# 克隆项目
git clone https://github.com/tianrking/robstride-control.git
cd robstride-control/rust

# 编译
cargo build --release

# 运行
cargo run --release -- 11
```

## 快速开始

### 基本使用

```rust
use socketcan::{CanSocket, CanFrame, EmbeddedFrame};
use std::sync::{Arc, Mutex};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let motor_id: u8 = 11;

    // 创建 CAN socket
    let socket = Arc::new(Mutex::new(
        CanSocket::open("can0")?
    ));

    // 启用电机
    enable_motor(&socket, motor_id)?;

    // 设置 MIT 模式
    set_mode_raw(&socket, motor_id, 0)?;

    // 发送位置指令
    write_operation_frame(&socket.lock()?, motor_id,
        std::f64::consts::PI / 2.0, // 90度
        30.0,                        // Kp
        0.5                          // Kd
    )?;

    Ok(())
}
```

### 编译运行

```bash
# 开发模式（快速编译）
cargo run -- 11

# 发布模式（优化性能）
cargo run --release -- 11

# 直接运行编译后的程序
./target/release/robstride-mit-position 11
```

## API 参考

### CAN 接口

```rust
// 创建 socket
let socket = CanSocket::open("can0")?;

// 发送帧
let can_id = CanId::extended(0x12345678)?;
let frame = CanFrame::new(can_id, &[0x01, 0x02, 0x03, 0x04])?;
socket.write_frame(&frame)?;

// 接收帧
socket.set_read_timeout(Duration::from_millis(100))?;
let frame = socket.read_frame()?;
```

### 电机控制函数

```rust
// 电机使能
fn enable_motor(socket_arc: &Arc<Mutex<CanSocket>>, motor_id: u8) -> Result<()>

// 模式设置
fn set_mode_raw(socket_arc: &Arc<Mutex<CanSocket>>, motor_id: u8, mode: i8) -> Result<()>

// 参数写入
fn write_limit(socket_arc: &Arc<Mutex<CanSocket>>, motor_id: u8,
               param_id: u16, limit: f32) -> Result<()>

// 操作帧写入
fn write_operation_frame(socket: &CanSocket, motor_id: u8,
                        pos: f64, kp_val: f64, kd_val: f64) -> Result<()>
```

### 协议常量

```rust
// 通信类型
const COMM_ENABLE: u32 = 3;
const COMM_OPERATION_CONTROL: u32 = 1;
const COMM_WRITE_PARAMETER: u32 = 18;

// 控制模式
const MODE_MIT: i8 = 0;
const MODE_POSITION: i8 = 1;
const MODE_SPEED: i8 = 2;

// 参数 ID
const PARAM_MODE: u16 = 0x7005;
const PARAM_VELOCITY_LIMIT: u16 = 0x7017;
const PARAM_TORQUE_LIMIT: u16 = 0x700B;
```

## 控制循环

### MIT 模式控制

```rust
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

// 共享状态
let running = Arc::new(AtomicBool::new(true));
let target_pos = Arc::new(AtomicU64::new(0));
let kp = Arc::new(AtomicU64::new(0));
let kd = Arc::new(AtomicU64::new(0));

// 控制线程
let loop_running = running.clone();
let loop_socket = socket.clone();
let loop_pos = target_pos.clone();
let loop_kp = kp.clone();
let loop_kd = kd.clone();

thread::spawn(move || {
    control_loop(
        loop_running,
        loop_socket,
        motor_id,
        loop_pos,
        loop_kp,
        loop_kd
    );
});

fn control_loop(
    running: Arc<AtomicBool>,
    socket_arc: Arc<Mutex<CanSocket>>,
    motor_id: u8,
    pos_arc: Arc<AtomicU64>,
    kp_arc: Arc<AtomicU64>,
    kd_arc: Arc<AtomicU64>
) {
    while running.load(Ordering::SeqCst) {
        // 获取当前参数
        let pos = f64::from_bits(pos_arc.load(Ordering::SeqCst));
        let kp = f64::from_bits(kp_arc.load(Ordering::SeqCst));
        let kd = f64::from_bits(kd_arc.load(Ordering::SeqCst));

        // 发送控制指令
        let socket = socket_arc.lock().unwrap();
        if let Err(e) = write_operation_frame(&socket, motor_id, pos, kp, kd) {
            eprintln!("发送错误: {}", e);
        }

        // 读取状态帧
        while read_operation_frame(&socket) {}

        // 控制频率 50Hz
        thread::sleep(Duration::from_millis(20));
    }
}
```

### 交互式控制

```rust
use std::io::{self, Write};

fn interactive_control(
    running: Arc<AtomicBool>,
    target_pos: Arc<AtomicU64>,
    kp: Arc<AtomicU64>,
    kd: Arc<AtomicU64>
) {
    while running.load(Ordering::SeqCst) {
        let pos_deg = f64::from_bits(target_pos.load(Ordering::SeqCst)).to_degrees();
        print!("[{:.1}°] >> ", pos_deg);
        io::stdout().flush().unwrap();

        let mut input = String::new();
        if io::stdin().read_line(&mut input).is_err() {
            break;
        }

        match input.trim() {
            "q" | "quit" | "exit" => running.store(false, Ordering::SeqCst),
            "0" | "home" => {
                target_pos.store(0.0f64.to_bits(), Ordering::SeqCst);
                println!(" -> 目标设定: 0.0°");
            }
            cmd if cmd.starts_with("kp ") => {
                if let Ok(val) = cmd[3..].trim().parse::<f64>() {
                    kp.store(val.to_bits(), Ordering::SeqCst);
                    println!(" -> Kp设定: {}", val);
                }
            }
            cmd if cmd.starts_with("kd ") => {
                if let Ok(val) = cmd[3..].trim().parse::<f64>() {
                    kd.store(val.to_bits(), Ordering::SeqCst);
                    println!(" -> Kd设定: {}", val);
                }
            }
            cmd => {
                if let Ok(angle_deg) = cmd.parse::<f64>() {
                    let angle_clamped = angle_deg.max(-720.0).min(720.0);
                    target_pos.store(angle_clamped.to_radians().to_bits(), Ordering::SeqCst);
                    println!(" -> 目标设定: {}°", angle_clamped);
                }
            }
        }
    }
}
```

## 错误处理

### 自定义错误类型

```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RobStrideError {
    #[error("CAN 接口错误: {0}")]
    CanError(#[from] socketcan::Error),

    #[error("无效的电机 ID: {0}")]
    InvalidMotorId(u8),

    #[error("参数超出范围: {0}")]
    ParameterOutOfRange(String),

    #[error("电机未连接")]
    MotorNotConnected,
}

type Result<T> = std::result::Result<T, RobStrideError>;
```

### 安全性保证

```rust
// 线程安全的状态访问
fn safe_update_position(
    pos_arc: &Arc<AtomicU64>,
    new_pos: f64
) -> Result<()> {
    if !new_pos.is_finite() {
        return Err(RobStrideError::ParameterOutOfRange(
            "位置必须是有限数值".to_string()
        ));
    }

    let clamped = new_pos.max(-2.0 * PI).min(2.0 * PI);
    pos_arc.store(clamped.to_bits(), Ordering::SeqCst);
    Ok(())
}
```

## 性能优化

### 编译优化

```toml
# Cargo.toml
[profile.release]
lto = true              # 链接时优化
codegen-units = 1       # 单个代码生成单元
panic = "abort"         # 直接 abort，减少二进制大小
opt-level = 3           # 最高优化级别
```

### 运行时优化

```rust
// 预分配缓冲区
let mut frame_buffer = Vec::with_capacity(1000);

// 避免动态分配
let mut data = [0u8; 8];

// 使用 unsafe 进行优化（在安全的前提下）
unsafe {
    std::ptr::write_unaligned(data.as_mut_ptr() as *mut f32, value);
}
```

## 测试

### 单元测试

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_can_frame_packing() {
        let pos = 1.0;
        let kp = 30.0;
        let kd = 0.5;

        // 测试帧打包
        let frame = pack_operation_frame(1, pos, kp, kd);
        assert!(frame.is_ok());
    }

    #[test]
    fn test_parameter_validation() {
        assert!(validate_kp(30.0).is_ok());
        assert!(validate_kp(-1.0).is_err());
        assert!(validate_kd(0.5).is_ok());
    }
}
```

### 集成测试

```bash
# 运行所有测试
cargo test

# 运行特定测试
cargo test test_can_frame

# 运行集成测试
cargo test --test integration_tests

# 运行示例程序
cargo run --bin basic_control -- 11
```

## 调试

### 日志记录

```rust
use log::{info, warn, error, debug};

// Cargo.toml
// [dependencies]
// log = "0.4"
// env_logger = "0.10"

fn main() {
    env_logger::init();

    info!("程序启动");
    debug!("连接到 CAN 接口");
    warn!("电机响应较慢");
    error!("通信失败");
}
```

### 性能分析

```bash
# 安装性能分析工具
cargo install cargo-flamegraph

# 生成火焰图
cargo flamegraph --bin robstride-mit-position -- 11

# CPU 性能分析
perf record --call-graph=dwarf ./target/release/robstride-mit-position 11
perf report
```

## 部署

### 静态链接

```bash
# 静态编译
cargo build --release --target x86_64-unknown-linux-musl

# 创建最小二进制
strip target/x86_64-unknown-linux-musl/release/robstride-mit-position
```

### Docker 部署

```dockerfile
FROM rust:1.70 as builder
WORKDIR /app
COPY . .
RUN cargo build --release

FROM debian:bullseye-slim
RUN apt-get update && apt-get install -y ca-certificates
COPY --from=builder /app/target/release/robstride-mit-position /usr/local/bin/
CMD ["robstride-mit-position"]
```

## 故障排除

### 编译错误

```bash
# 更新 Rust
rustup update

# 清理缓存
cargo clean

# 检查依赖
cargo tree
```

### 运行时错误

```bash
# 启用详细日志
RUST_LOG=debug cargo run -- 11

# 检查 CAN 权限
sudo chmod 666 /dev/can0

# 检查 CAN 状态
ip -details link show can0
```

## 许可证

MIT License - 详见 [LICENSE](../LICENSE) 文件

## 贡献

欢迎提交 Issue 和 Pull Request！

## 支持

- 📖 [完整文档](../docs/)
- 🐛 [问题反馈](https://github.com/tianrking/robstride-control/issues)