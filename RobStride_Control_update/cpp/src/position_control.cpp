/*
 * RobStride MIT 模式位置控制 (C++ 版本)
 * 模式: Mode 0 (MIT Mode)
 * 通信: 循环调用 write_operation_frame
 *
 * 编译:
 * g++ -o position_control_mit position_control_mit.cpp -lpthread -std=c++17
 *
 * 运行:
 * sudo ./position_control_mit <motor_id>
 * (需要 sudo 权限来访问 CAN 硬件)
 */

#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <csignal>
#include <cmath>

// Linux SocketCAN 头文件
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// --- 全局原子变量，用于线程间安全通信 ---
std::atomic<bool> running(true);
std::atomic<double> target_position(0.0);
std::atomic<double> kp(100.0); // 高刚度
std::atomic<double> kd(2.0);  // 高阻尼

// --- 电机和协议定义 ---
const int MOTOR_ID_DEFAULT = 11; // 默认电机ID
const char* CAN_INTERFACE = "can0";
const int HOST_ID = 0xFF;

// 通信类型
const uint32_t COMM_OPERATION_CONTROL = 1;
const uint32_t COMM_ENABLE = 3;
const uint32_t COMM_WRITE_PARAMETER = 18;

// 参数 ID
const uint16_t PARAM_MODE = 0x7005;
const uint16_t PARAM_VELOCITY_LIMIT = 0x7017;
const uint16_t PARAM_TORQUE_LIMIT = 0x700B; // 注意: 根据 protocol.py 应该是 0x700B

// --- CAN 帧打包辅助函数 ---

// 将 float 复制到 uint8_t 数组中 (小端)
void pack_float_le(uint8_t* buf, float val) {
    memcpy(buf, &val, sizeof(float));
}

// 将 uint16_t 复制到 uint8_t 数组中 (小端)
void pack_u16_le(uint8_t* buf, uint16_t val) {
    memcpy(buf, &val, sizeof(uint16_t));
}

// 将 uint16_t 打包为大端字节序
void pack_u16_be(uint8_t* buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

// --- 底层 CAN 函数 ---

/**
 * @brief 发送一个 CAN 帧
 */
bool send_frame(int s, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG; // 启用扩展帧 (29-bit)
    frame.can_dlc = dlc;
    if (data) {
        memcpy(frame.data, data, dlc);
    } else {
        memset(frame.data, 0, 8);
    }

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("write");
        return false;
    }
    return true;
}

/**
 * @brief 读取一个 CAN 帧 (带超时)
 */
bool read_frame(int s, struct can_frame* frame) {
    // 设置 100ms 超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(s, &rdfs);

    int ret = select(s + 1, &rdfs, NULL, NULL, &tv);
    if (ret == -1) {
        perror("select");
        return false;
    } else if (ret == 0) {
        // std::cerr << "read timeout" << std::endl;
        return false; // 超时
    }

    if (read(s, frame, sizeof(struct can_frame)) < 0) {
        perror("read");
        return false;
    }
    return true;
}

// --- RobStride 协议函数 ---

bool enable_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool set_mode_raw(int s, int motor_id, int8_t mode) {
    uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], PARAM_MODE); // param ID
    data[4] = (uint8_t)mode;           // value (int8)
    return send_frame(s, ext_id, data, 8);
}

bool write_limit(int s, int motor_id, uint16_t param_id, float limit) {
    uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], param_id); // param ID
    pack_float_le(&data[4], limit);  // value (float)
    return send_frame(s, ext_id, data, 8);
}

bool write_operation_frame(int s, int motor_id, double pos, double kp_val, double kd_val) {
    // 1. 打包数据 (大端序!)
    // 这些 scaling value 应该从 table.py 导入，这里为了简化硬编码
    const double POS_SCALE = 4 * M_PI; // rs-03
    const double VEL_SCALE = 50.0;     // rs-03
    const double KP_SCALE = 5000.0;    // rs-03
    const double KD_SCALE = 100.0;     // rs-03
    const double TQ_SCALE = 60.0;      // rs-03

    // 裁剪和转换
    double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
    double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
    double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));
    
    uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
    uint16_t vel_u16 = 0x7FFF; // 0 velocity
    uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
    uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
    uint16_t torque_u16 = 0x7FFF; // 0 torque_ff

    uint8_t data[8];
    pack_u16_be(&data[0], pos_u16);
    pack_u16_be(&data[2], vel_u16);
    pack_u16_be(&data[4], kp_u16);
    pack_u16_be(&data[6], kd_u16);
    
    // 2. 构建 CAN ID
    uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
    
    // 3. 发送
    return send_frame(s, ext_id, data, 8);
}

bool read_operation_frame(int s) {
    struct can_frame frame;
    if (read_frame(s, &frame)) {
        if (!frame.can_id & CAN_EFF_FLAG) return false;
        
        uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
        if (comm_type == 2) { // 状态包
            return true;
        }
    }
    return false;
}

/**
 * @brief 控制循环线程
 */
void control_loop(int s, int motor_id) {
    std::cout << "🔄 控制循环已启动 (Mode 0 @ 50Hz)" << std::endl;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // 1. 发送 MIT 帧 (只发)
        write_operation_frame(s, motor_id, target_position.load(), kp.load(), kd.load());
        
        // 2. 读取状态帧 (只收)，清空缓冲区
        while(read_operation_frame(s)); // 循环读取，直到缓冲区为空或超时
        
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        // 固定的 50Hz 循环
        std::this_thread::sleep_for(std::chrono::microseconds(20000) - elapsed);
    }
    std::cout << "⏹️ 控制线程停止" << std::endl;
}

/**
 * @brief 初始化 SocketCAN
 */
int init_can(const char* ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    return s;
}

void signal_handler(int signum) {
    std::cout << "\n🛑 捕获到退出信号..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    int motor_id = MOTOR_ID_DEFAULT;
    if (argc > 1) {
        motor_id = std::atoi(argv[1]);
    }
    
    std::cout << "🎯 MIT 位置控制台 (ID: " << motor_id << ")" << std::endl;
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int s = init_can(CAN_INTERFACE);
    if (s < 0) {
        std::cerr << "❌ 无法打开 CAN 接口 " << CAN_INTERFACE << std::endl;
        return 1;
    }
    std::cout << "📡 CAN 总线 " << CAN_INTERFACE << " 连接成功" << std::endl;

    // --- 初始化电机 ---
    std::cout << "⚡ 激活电机 ID: " << motor_id << " ..." << std::endl;
    enable_motor(s, motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "⚙️ 切换到 MIT 模式 (Mode 0)..." << std::endl;
    set_mode_raw(s, motor_id, 0);
    
    std::cout << "⚙️ 设置内部限制..." << std::endl;
    write_limit(s, motor_id, PARAM_VELOCITY_LIMIT, 20.0);
    write_limit(s, motor_id, PARAM_TORQUE_LIMIT, 20.0);
    
    std::cout << "🏠 设置初始目标为 0.0 ..." << std::endl;
    write_operation_frame(s, motor_id, 0.0, kp.load(), kd.load());
    std::cout << "✅ 初始化完成！" << std::endl;

    // 启动控制线程
    std::thread t(control_loop, s, motor_id);

    // --- 交互式主循环 ---
    std::cout << "\n" << "========================================" << std::endl;
    std::cout << "👉 输入数字 (度) 回车即可改变位置" << std::endl;
    std::cout << "👉 'kp <值>' (例如: kp 100) 来调节刚度" << std::endl;
    std::cout << "👉 'kd <值>' (例如: kd 2.0) 来调节阻尼 (防抖)" << std::endl;
    std::cout << "👉 '0' 或 'home' 回到零点" << std::endl;
    std::cout << "👉 'q' 退出" << std::endl;
    std::cout << "⚠️  当前 Kp=" << kp.load() << " | Kd=" << kd.load() << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::string line;
    while (running) {
        double current_pos_deg = target_position.load() * 180.0 / M_PI;
        printf("[%.1f°] >> ", current_pos_deg);
        std::flush(std::cout);

        if (!std::getline(std::cin, line)) {
            break;
        }

        if (line == "q" || line == "quit" || line == "exit") {
            running = false;
        } else if (line == "0" || line == "home") {
            target_position = 0.0;
            std::cout << " -> 目标设定: 0.0°" << std::endl;
        } else if (line.rfind("kp ", 0) == 0) {
            try {
                kp = std::stod(line.substr(3));
                std::cout << " -> 刚度(Kp)设定: " << kp.load() << std::endl;
            } catch (...) {
                std::cout << "❌ 无效 Kp. 示例: kp 100.0" << std::endl;
            }
        } else if (line.rfind("kd ", 0) == 0) {
            try {
                kd = std::stod(line.substr(3));
                std::cout << " -> 阻尼(Kd)设定: " << kd.load() << std::endl;
            } catch (...) {
                std::cout << "❌ 无效 Kd. 示例: kd 2.0" << std::endl;
            }
        } else {
            try {
                double angle_deg = std::stod(line);
                angle_deg = std::max(-720.0, std::min(720.0, angle_deg));
                target_position = angle_deg * M_PI / 180.0;
                std::cout << " -> 目标设定: " << angle_deg << "°" << std::endl;
            } catch (...) {
                std::cout << "❌ 无效输入" << std::endl;
            }
        }
    }

    // 清理
    running = false;
    t.join();
    
    std::cout << "🏠 回到零位..." << std::endl;
    write_operation_frame(s, motor_id, 0.0, kp.load(), kd.load());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    std::cout << "🚫 禁用电机..." << std::endl;
    // C++ SDK 中没有 disable, 我们手动发送一个 kp=0, kd=0 的帧
    write_operation_frame(s, motor_id, 0.0, 0.0, 0.0);
    
    close(s);
    std::cout << "👋 程序结束" << std::endl;
    
    return 0;
}