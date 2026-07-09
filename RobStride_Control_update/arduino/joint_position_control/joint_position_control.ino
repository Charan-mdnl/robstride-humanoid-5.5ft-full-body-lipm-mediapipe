/*
 * 小米电机关节位置控制程序 - ESP32版本
 *
 * 功能：
 * - 专门用于关节电机控制
 * - 串口透传位置命令
 * - 平滑运动控制，减少振动
 * - 实时位置反馈
 *
 * 串口命令格式：
 * - "pos 1.57"     # 转到1.57弧度位置
 * - "angle 90"     # 转到90度位置
 * - "stop"         # 停止电机
 * - "zero"         # 设置当前位置为零点
 * - "status"       # 显示当前状态
 * - "help"         # 显示帮助
 *
 * 硬件连接：
 * - ESP32开发板
 * - CAN模块：TX->GPIO5, RX->GPIO4
 * - 小米电机：CAN_H, CAN_L, 电源(12-24V)
 *
 * 作者：基于小米电机通信协议实现
 * 日期：2024年
 */

#include <Arduino.h>
#include "TWAI_CAN_MI_Motor.h"

// 创建电机控制对象
MI_Motor_ M1_con;

// 控制参数
float target_position = 0.0;    // 目标位置 (rad)
float current_position = 0.0;   // 当前位置 (rad)
float max_speed = 3.0;          // 最大速度 (rad/s)
float position_tolerance = 0.05; // 位置容差 (rad)

// 状态变量
bool motor_enabled = false;
bool position_mode = true;
bool moving_to_target = false;
unsigned long last_update_time = 0;
unsigned long last_command_time = 0;

// 平滑控制参数
float kp = 30.0;                // 位置控制比例增益
float kd = 2.0;                 // 位置控制微分增益

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== 小米电机关节位置控制程序 ESP32 ===");
    Serial.println("专为关节电机优化，支持串口透传控制");
    Serial.println();

    // 初始化CAN总线
    Serial.println("1. 初始化CAN总线...");
    Motor_CAN_Init();
    delay(100);

    // 初始化电机
    Serial.println("2. 初始化电机(ID=1)...");
    M1_con.Motor_Con_Init(MOTER_1_ID);
    delay(100);

    // 设置电机零点
    Serial.println("3. 设置电机机械零点...");
    M1_con.Motor_Set_Zero();
    delay(500);

    // 切换到位置模式
    Serial.println("4. 设置位置控制模式...");
    M1_con.Change_Mode(POS_MODE);
    delay(200);

    // 设置位置控制参数（减少振动）
    Serial.println("5. 配置位置控制参数...");
    configure_position_control();
    delay(200);

    // 使能电机
    Serial.println("6. 使能电机...");
    M1_con.Motor_Enable();
    delay(500);

    // 设置初始位置为0
    set_position(0.0);
    delay(200);

    motor_enabled = true;
    last_update_time = millis();
    last_command_time = millis();

    Serial.println("=== 系统初始化完成 ===");
    Serial.println("电机已使能，等待位置命令...");
    Serial.println();
    print_help();
    Serial.println("准备就绪，可以输入位置命令:");
    Serial.println("例如：pos 1.57 或 angle 90");
    Serial.println();
}

void loop() {
    unsigned long current_time = millis();

    // 定期更新电机数据
    if (current_time - last_update_time >= 50) {  // 20Hz更新率
        last_update_time = current_time;
        update_motor_status();
    }

    // 处理串口命令
    if (Serial.available()) {
        handle_serial_command();
        last_command_time = current_time;
    }

    // 10秒无命令自动停止
    if (motor_enabled && (current_time - last_command_time > 10000)) {
        if (moving_to_target) {
            Serial.println("长时间无命令，停止运动");
            emergency_stop();
        }
    }

    delay(10);  // 小延时，防止CPU占用过高
}

void configure_position_control() {
    // 设置位置控制参数以减少振动
    Serial.println("配置位置模式参数...");

    // 设置最大速度限制（减少振动）
    M1_con.Set_SingleParameter(LIMIT_SPD, max_speed);
    delay(50);

    // 设置位置控制的KP参数（较小的值减少振动）
    M1_con.Set_SingleParameter(LOC_KP, kp);
    delay(50);

    // 设置速度控制的KP参数
    M1_con.Set_SingleParameter(SPD_KP, 10.0);
    delay(50);

    // 设置速度控制的KI参数
    M1_con.Set_SingleParameter(SPD_KI, 0.1);
    delay(50);

    // 设置电流限制
    M1_con.Set_SingleParameter(LIMIT_CUR, 5.0);
    delay(50);

    Serial.printf("位置控制参数已配置：最大速度=%.1f rad/s, KP=%.1f\r\n", max_speed, kp);
}

void set_position(float position) {
    // 限制位置范围
    if (position > P_MAX) position = P_MAX;
    if (position < P_MIN) position = P_MIN;

    target_position = position;
    moving_to_target = true;

    Serial.printf("设置目标位置：%.3f rad (%.1f°)\r\n", position, position * 180.0 / PI);

    // 分两步设置：先设置速度限制，再设置位置
    M1_con.Set_SingleParameter(LIMIT_SPD, max_speed);
    delay(20);
    M1_con.Set_SingleParameter(LOC_REF, position);
}

void update_motor_status() {
    uint8_t result = M1_con.Motor_Data_Updata(20);

    if (result == 0) {
        // 成功读取数据
        can_rx_frame_t* data = &M1_con.motor_rx_data;
        current_position = data->cur_angle;

        // 检查是否到达目标位置
        if (moving_to_target) {
            float position_error = abs(current_position - target_position);
            if (position_error < position_tolerance) {
                moving_to_target = false;
                Serial.printf("✓ 到达目标位置！误差: %.3f rad\r\n", position_error);
            }
        }

        // 简化状态显示
        if (millis() % 1000 < 50) {  // 每秒显示一次
            print_compact_status();
        }

        // 检查错误
        check_motor_errors(data);
    }
}

void print_compact_status() {
    can_rx_frame_t* data = &M1_con.motor_rx_data;

    Serial.printf("状态: 位置=%.3f rad (%.1f°), 速度=%.2f rad/s, 扭矩=%.2f Nm, 温度=%.1f°C",
                  data->cur_angle, data->cur_angle * 180.0 / PI,
                  data->cur_speed, data->cur_torque, data->cur_temp);

    if (moving_to_target) {
        float error = abs(target_position - current_position);
        Serial.printf(", 目标误差=%.3f rad", error);
    }

    Serial.println();
}

void check_motor_errors(can_rx_frame_t* data) {
    if (data->err_sta) {
        Serial.print("⚠️ 电机错误：");
        if (data->HALL_err) Serial.print("HALL ");
        if (data->magnet_err) Serial.print("磁编码器 ");
        if (data->temp_err) Serial.print("温度 ");
        if (data->current_err) Serial.print("电流 ");
        if (data->voltage_err) Serial.print("电压 ");
        Serial.println();
    }

    if (data->cur_temp > 70.0) {
        Serial.printf("🌡️ 高温警告：%.1f°C\r\n", data->cur_temp);
    }
}

void handle_serial_command() {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) return;

    Serial.print("收到命令: ");
    Serial.println(command);

    if (command == "stop") {
        stop_motor();

    } else if (command == "zero") {
        set_zero_position();

    } else if (command == "enable") {
        enable_motor();

    } else if (command == "disable") {
        disable_motor();

    } else if (command == "status") {
        show_detailed_status();

    } else if (command.startsWith("pos ")) {
        float pos = parse_position_command(command, "pos ");
        if (pos != NAN) {
            set_position(pos);
        }

    } else if (command.startsWith("angle ")) {
        float angle = parse_position_command(command, "angle ");
        if (angle != NAN) {
            float pos = angle * PI / 180.0;  // 角度转弧度
            set_position(pos);
        }

    } else if (command.startsWith("speed ")) {
        float speed = parse_speed_command(command);
        if (speed != NAN) {
            set_max_speed(speed);
        }

    } else if (command == "help") {
        print_help();

    } else {
        Serial.println("未知命令，输入'help'查看帮助");
    }
}

float parse_position_command(String command, String prefix) {
    String value_str = command.substring(prefix.length());
    value_str.trim();

    float value = value_str.toFloat();
    if (abs(value) <= 1000) {  // 合理范围检查
        return value;
    } else {
        Serial.println("❌ 位置值超出合理范围");
        return NAN;
    }
}

float parse_speed_command(String command) {
    String value_str = command.substring(6);  // "speed " 长度为6
    value_str.trim();

    float speed = value_str.toFloat();
    if (speed > 0 && speed <= 30.0) {
        return speed;
    } else {
        Serial.println("❌ 速度值范围应为 0-30 rad/s");
        return NAN;
    }
}

void set_max_speed(float speed) {
    max_speed = speed;
    M1_con.Set_SingleParameter(LIMIT_SPD, max_speed);
    Serial.printf("最大速度设置为: %.1f rad/s\r\n", max_speed);
}

void set_zero_position() {
    Serial.println("设置当前位置为零点...");
    M1_con.Motor_Set_Zero();
    delay(500);

    // 重新配置参数
    configure_position_control();

    target_position = 0.0;
    current_position = 0.0;
    moving_to_target = false;

    Serial.println("✓ 零点设置完成");
}

void stop_motor() {
    Serial.println("停止电机...");
    M1_con.Set_SingleParameter(LOC_REF, current_position);  // 保持当前位置
    moving_to_target = false;
    Serial.println("✓ 电机已停止");
}

void emergency_stop() {
    Serial.println("紧急停止！");
    M1_con.Motor_Reset();
    delay(100);
    M1_con.Motor_Enable();
    delay(100);
    M1_con.Change_Mode(POS_MODE);
    delay(100);
    configure_position_control();
    moving_to_target = false;
}

void enable_motor() {
    if (!motor_enabled) {
        Serial.println("使能电机...");
        M1_con.Motor_Enable();
        motor_enabled = true;
        delay(200);
        Serial.println("✓ 电机已使能");
    } else {
        Serial.println("电机已经使能");
    }
}

void disable_motor() {
    if (motor_enabled) {
        Serial.println("禁用电机...");
        M1_con.Motor_Reset();
        motor_enabled = false;
        moving_to_target = false;
        Serial.println("✓ 电机已禁用");
    } else {
        Serial.println("电机已经禁用");
    }
}

void show_detailed_status() {
    can_rx_frame_t* data = &M1_con.motor_rx_data;

    Serial.println("=== 详细状态信息 ===");
    Serial.printf("电机ID: %d, 主站ID: %d\r\n", data->motor_id, data->master_id);
    Serial.printf("当前位置: %.3f rad (%.2f°)\r\n", data->cur_angle, data->cur_angle * 180.0 / PI);
    Serial.printf("目标位置: %.3f rad (%.2f°)\r\n", target_position, target_position * 180.0 / PI);
    Serial.printf("位置误差: %.3f rad (%.2f°)\r\n",
                  abs(data->cur_angle - target_position),
                  abs(data->cur_angle - target_position) * 180.0 / PI);
    Serial.printf("当前速度: %.3f rad/s (%.1f RPM)\r\n", data->cur_speed, data->cur_speed * 60.0 / (2 * PI));
    Serial.printf("当前扭矩: %.3f Nm\r\n", data->cur_torque);
    Serial.printf("电机温度: %.1f°C\r\n", data->cur_temp);
    Serial.printf("最大速度限制: %.1f rad/s\r\n", max_speed);
    Serial.printf("运行状态: %s\r\n", motor_enabled ? "使能" : "禁用");
    Serial.printf("运动状态: %s\r\n", moving_to_target ? "运动中" : "静止");

    Serial.print("错误状态: ");
    if (data->err_sta) {
        Serial.print("有错误 - ");
        if (data->HALL_err) Serial.print("HALL ");
        if (data->magnet_err) Serial.print("磁编码器 ");
        if (data->temp_err) Serial.print("温度 ");
        if (data->current_err) Serial.print("电流 ");
        if (data->voltage_err) Serial.print("电压 ");
    } else {
        Serial.print("正常");
    }
    Serial.println();
    Serial.println("==================");
}

void print_help() {
    Serial.println("=== 串口命令帮助 ===");
    Serial.println("位置控制命令:");
    Serial.println("  pos X.XX     - 转到X.XX弧度位置 (范围: -12.5 到 12.5)");
    Serial.println("  angle X.XX   - 转到X.XX度位置 (范围: -720 到 720)");
    Serial.println("  speed X.X    - 设置最大速度X.X rad/s (范围: 0 到 30)");
    Serial.println();
    Serial.println("控制命令:");
    Serial.println("  stop         - 停止电机运动");
    Serial.println("  zero         - 设置当前位置为零点");
    Serial.println("  enable       - 使能电机");
    Serial.println("  disable      - 禁用电机");
    Serial.println();
    Serial.println("状态命令:");
    Serial.println("  status       - 显示详细状态信息");
    Serial.println("  help         - 显示此帮助信息");
    Serial.println();
    Serial.println("示例:");
    Serial.println("  pos 1.57     # 转到1.57弧度(90度)");
    Serial.println("  angle -90    # 转到-90度位置");
    Serial.println("  speed 5.0    # 设置最大速度5 rad/s");
    Serial.println("  stop         # 立即停止");
    Serial.println();
}

// 程序结束时的清理工作
void cleanup() {
    if (motor_enabled) {
        Serial.println("正在安全停止电机...");
        emergency_stop();
        delay(200);
    }
    Serial.println("程序结束");
}