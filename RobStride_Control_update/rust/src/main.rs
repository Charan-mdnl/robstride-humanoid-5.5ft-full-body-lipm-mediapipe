/*
 * RobStride MIT 模式位置控制 (Rust 版本)
 * 模式: Mode 0 (MIT Mode)
 * 通信: 循环调用 write_operation_frame
 *
 * 修复: 解决了多线程 Socket 冲突问题。
 * 现在所有线程共享一个 Arc<Mutex<CanSocket>>。
 *
 * 2. 编辑 `Cargo.toml` 并添加依赖 (使用 v3.x API):
 * [dependencies]
 * socketcan = "3.5.0"
 * ctrlc = "3.5.1"
 */

// --- 修复 1: 导入 'EmbeddedFrame' (用于 .new(), .id(), .is_extended()) ---
use socketcan::{CanSocket, CanFrame, Socket, EmbeddedFrame, CanId};
use std::io::{self, Write};
use std::thread;
use std::time::Duration;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex}; // <-- 修复 2: 导入 Mutex
use std::f64::consts::PI;

// --- 辅助函数来安全地读写 f64 (存储在 AtomicU64 中) ---
fn set_atomic_f64(atom: &AtomicU64, val: f64) {
    atom.store(val.to_bits(), Ordering::SeqCst);
}

fn get_atomic_f64(atom: &AtomicU64) -> f64 {
    f64::from_bits(atom.load(Ordering::SeqCst))
}


// --- 电机和协议定义 ---
const CAN_INTERFACE: &str = "can0";
const HOST_ID: u32 = 0xFF;

// 通信类型
const COMM_OPERATION_CONTROL: u32 = 1;
const COMM_ENABLE: u32 = 3;
const COMM_WRITE_PARAMETER: u32 = 18;

// 参数 ID
const PARAM_MODE: u16 = 0x7005;
const PARAM_VELOCITY_LIMIT: u16 = 0x7017;
const PARAM_TORQUE_LIMIT: u16 = 0x700B;

// --- RobStride 协议函数 ---
// --- 修复 3: 所有总线函数现在都接收 Arc<Mutex<...>> ---

fn enable_motor(socket_arc: &Arc<Mutex<CanSocket>>, motor_id: u8) -> socketcan::Result<()> {
    let socket = socket_arc.lock().unwrap(); // <-- 获取锁
    let ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | (motor_id as u32);
    
    let can_id = CanId::extended(ext_id).unwrap();
    let frame = CanFrame::new(can_id, &[]).unwrap();
    
    socket.write_frame(&frame)?;
    Ok(())
} // <-- 锁在这里自动释放

fn set_mode_raw(socket_arc: &Arc<Mutex<CanSocket>>, motor_id: u8, mode: i8) -> socketcan::Result<()> {
    let socket = socket_arc.lock().unwrap(); // <-- 获取锁
    let ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | (motor_id as u32);
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&PARAM_MODE.to_le_bytes());
    data[4] = mode as u8;

    let can_id = CanId::extended(ext_id).unwrap();
    let frame = CanFrame::new(can_id, &data).unwrap();
    
    socket.write_frame(&frame)?;
    Ok(())
}

fn write_limit(socket_arc: &Arc<Mutex<CanSocket>>, motor_id: u8, param_id: u16, limit: f32) -> socketcan::Result<()> {
    let socket = socket_arc.lock().unwrap(); // <-- 获取锁
    let ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | (motor_id as u32);
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&param_id.to_le_bytes());
    data[4..8].copy_from_slice(&limit.to_le_bytes());
    
    let can_id = CanId::extended(ext_id).unwrap();
    let frame = CanFrame::new(can_id, &data).unwrap();
    
    socket.write_frame(&frame)?;
    Ok(())
}

fn write_operation_frame(socket: &CanSocket, motor_id: u8, pos: f64, kp_val: f64, kd_val: f64) -> socketcan::Result<()> {
    // 1. 打包数据 (大端序!)
    const POS_SCALE: f64 = 4.0 * PI; // rs-03
    const KP_SCALE: f64 = 5000.0;    // rs-03
    const KD_SCALE: f64 = 100.0;     // rs-03
    
    // 裁剪和转换
    let pos_clamped = pos.max(-POS_SCALE).min(POS_SCALE);
    let kp_clamped = kp_val.max(0.0).min(KP_SCALE);
    let kd_clamped = kd_val.max(0.0).min(KD_SCALE);
    
    let pos_u16 = (((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF as f64) as u16;
    let vel_u16 = 0x7FFF; // 0 velocity
    let kp_u16 = ((kp_clamped / KP_SCALE) * 0xFFFF as f64) as u16;
    let kd_u16 = ((kd_clamped / KD_SCALE) * 0xFFFF as f64) as u16;
    let torque_u16 = 0x7FFF; // 0 torque_ff

    let data: [u8; 8] = [
        (pos_u16 >> 8) as u8, (pos_u16 & 0xFF) as u8,
        (vel_u16 >> 8) as u8, (vel_u16 & 0xFF) as u8,
        (kp_u16 >> 8) as u8, (kp_u16 & 0xFF) as u8,
        (kd_u16 >> 8) as u8, (kd_u16 & 0xFF) as u8,
    ];
    
    // 2. 构建 CAN ID
    let ext_id = (COMM_OPERATION_CONTROL << 24) | ((torque_u16 as u32) << 8) | (motor_id as u32);
    
    // 3. 发送
    let can_id = CanId::extended(ext_id).unwrap();
    let frame = CanFrame::new(can_id, &data).unwrap();

    socket.write_frame(&frame)?;
    Ok(())
}

// --- 修复 9: read_operation_frame 现在返回 bool ---
// (true = 读到一个帧, false = 超时/错误)
fn read_operation_frame(socket: &CanSocket) -> bool {
    // 注意：此函数在已锁定的 Mutex 内部被调用
    socket.set_read_timeout(Duration::from_millis(10)).unwrap_or_default();
    match socket.read_frame() {
        Ok(frame) => {
            if frame.is_extended() {
                if let socketcan::Id::Extended(id) = frame.id() {
                    let id_val: u32 = id.as_raw();
                    let comm_type = (id_val >> 24) & 0x1F; 
                    if comm_type == 2 {
                        return true; // 成功读到一个状态包
                    }
                }
            }
            true // 读到了一个非状态包
        },
        Err(e) => {
            if e.kind() != io::ErrorKind::WouldBlock {
                eprintln!("⚠️ 读取错误: {}", e);
            }
            false // 超时或错误
        }
    }
}

/**
 * @brief 控制循环线程
 */
// --- 修复 4: 传递所有共享状态的 Arc ---
fn control_loop(
    running: Arc<AtomicBool>,
    socket_arc: Arc<Mutex<CanSocket>>,
    motor_id: u8,
    pos_arc: Arc<AtomicU64>,
    kp_arc: Arc<AtomicU64>,
    kd_arc: Arc<AtomicU64>
) {
    println!("🔄 控制循环已启动 (Mode 0 @ 50Hz)");
    
    while running.load(Ordering::SeqCst) {
        let start = std::time::Instant::now();

        let pos = get_atomic_f64(&pos_arc);
        let kp = get_atomic_f64(&kp_arc);
        let kd = get_atomic_f64(&kd_arc);

        { // --- 修复 5: 为总线操作创建锁定范围 ---
            let socket = socket_arc.lock().unwrap();

            // 1. 发送 MIT 帧 (只发)
            if let Err(e) = write_operation_frame(&socket, motor_id, pos, kp, kd) {
                eprintln!("⚠️ 发送错误: {}", e);
            }
            
            // --- 修复 10: 循环, 直到 read_operation_frame 返回 false (超时) ---
            while read_operation_frame(&socket) {
                // 持续清空缓冲区
            }
        } // <-- 锁在这里自动释放
        
        // 固定的 50Hz 循环
        if let Some(sleep_time) = Duration::from_millis(20).checked_sub(start.elapsed()) {
            thread::sleep(sleep_time);
        }
    }
    println!("⏹️ 控制线程停止");
}

fn main() {
    let motor_id: u8 = std::env::args().nth(1)
        .unwrap_or("11".to_string())
        .parse()
        .expect("无效的 Motor ID");

    println!("🎯 MIT 位置控制台 (ID: {})", motor_id);
    
    // --- 修复 6: 创建 Arc 状态，而不是 static ---
    let running = Arc::new(AtomicBool::new(true));
    let target_position = Arc::new(AtomicU64::new(0));
    let kp = Arc::new(AtomicU64::new(0));
    let kd = Arc::new(AtomicU64::new(0));

    // --- 修复 7: 创建唯一的、带 Mutex 的 Socket ---
    let socket_arc = Arc::new(Mutex::new(
        CanSocket::open(CAN_INTERFACE).expect("❌ 无法打开 CAN 接口")
    ));
    println!("📡 CAN 总线 {} 连接成功", CAN_INTERFACE);
    
    // --- 设置 Ctrl-C 处理器 ---
    let r_clone = running.clone();
    ctrlc::set_handler(move || {
        println!("\n🛑 捕获到退出信号...");
        r_clone.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    // --- 初始化电机 ---
    { // 锁定 Socket 进行初始化
        println!("⚡ 激活电机 ID: {} ...", motor_id);
        enable_motor(&socket_arc, motor_id).unwrap();
        thread::sleep(Duration::from_millis(500));
        
        println!("⚙️ 切换到 MIT 模式 (Mode 0)...");
        set_mode_raw(&socket_arc, motor_id, 0).unwrap();
        
        println!("⚙️ 设置内部限制...");
        write_limit(&socket_arc, motor_id, PARAM_VELOCITY_LIMIT, 20.0).unwrap();
        write_limit(&socket_arc, motor_id, PARAM_TORQUE_LIMIT, 20.0).unwrap();
        
        // --- 设置初始参数 ---
        let init_kp = 100.0;
        let init_kd = 2.0;
        set_atomic_f64(&target_position, 0.0);
        set_atomic_f64(&kp, init_kp);
        set_atomic_f64(&kd, init_kd);

        println!("🏠 设置初始目标为 0.0 ...");
        // 必须显式锁定以调用
        let socket = socket_arc.lock().unwrap();
        write_operation_frame(&socket, motor_id, 0.0, init_kp, init_kd).unwrap();
        println!("✅ 初始化完成！");
    } // <-- 初始化完成，锁释放

    // --- 修复 8: 克隆所有 Arc 并移入线程 ---
    let loop_running = running.clone();
    let loop_pos = target_position.clone();
    let loop_kp = kp.clone();
    let loop_kd = kd.clone();
    let loop_socket = socket_arc.clone();
    let control_handle = thread::spawn(move || {
        control_loop(loop_running, loop_socket, motor_id, loop_pos, loop_kp, loop_kd);
    });

    // --- 交互式主循环 ---
    println!("\n========================================");
    println!("👉 输入数字 (度) 回车即可改变位置");
    println!("👉 'kp <值>' (例如: kp 100) 来调节刚度");
    println!("👉 'kd <值>' (例如: kd 2.0) 来调节阻尼 (防抖)");
    println!("👉 '0' 或 'home' 回到零点");
    println!("👉 'q' 退出");
    println!("⚠️  当前 Kp={} | Kd={}", get_atomic_f64(&kp), get_atomic_f64(&kd));
    println!("----------------------------------------");

    while running.load(Ordering::SeqCst) {
        let pos_deg = get_atomic_f64(&target_position).to_degrees();
        print!("[{:.1}°] >> ", pos_deg);
        io::stdout().flush().unwrap();

        let mut input = String::new();
        if io::stdin().read_line(&mut input).is_err() {
            break;
        }
        let cmd = input.trim();

        if cmd == "q" || cmd == "quit" || cmd == "exit" {
            running.store(false, Ordering::SeqCst);
        } else if cmd == "0" || cmd == "home" {
            set_atomic_f64(&target_position, 0.0);
            println!(" -> 目标设定: 0.0°");
        } else if cmd.starts_with("kp ") {
            if let Ok(val) = cmd[3..].trim().parse::<f64>() {
                set_atomic_f64(&kp, val);
                println!(" -> G(Kp)设定: {}", val);
            } else {
                println!("❌ 无效 Kp. 示例: kp 100.0");
            }
        } else if cmd.starts_with("kd ") {
            if let Ok(val) = cmd[3..].trim().parse::<f64>() {
                set_atomic_f64(&kd, val);
                println!(" -> 阻尼(Kd)设定: {}", val);
            } else {
                println!("❌ 无效 Kd. 示例: kd 2.0");
            }
        } else if let Ok(angle_deg) = cmd.parse::<f64>() {
            let angle_clamped = angle_deg.max(-720.0).min(720.0);
            set_atomic_f64(&target_position, angle_clamped.to_radians());
            println!(" -> 目标设定: {}°", angle_clamped);
        } else {
            if !cmd.is_empty() {
                println!("❌ 无效输入");
            }
        }
    }

    // 清理
    running.store(false, Ordering::SeqCst); // 确保线程停止
    control_handle.join().unwrap();
    
    { // 锁定 Socket 进行清理
        let socket = socket_arc.lock().unwrap();
        println!("🏠 回到零位...");
        write_operation_frame(&socket, motor_id, 0.0, get_atomic_f64(&kp), get_atomic_f64(&kd)).unwrap();
        thread::sleep(Duration::from_millis(1000));
        
        println!("🚫 禁用电机...");
        write_operation_frame(&socket, motor_id, 0.0, 0.0, 0.0).unwrap();
    } // <-- 锁释放
    
    println!("👋 程序结束");
}