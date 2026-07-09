/*
 * 小米电机控制程序 - ESP32版本
 *
 * 功能：
 * - 控制小米电机ID=1
 * - 支持速度、位置、电流、运控四种模式
 * - 实时读取电机状态数据
 *
 * 硬件连接：
 * - ESP32开发板
 * - CAN模块：TX->GPIO5, RX->GPIO4
 * - 小米电机：CAN_H, CAN_L, 电源(12-24V)
 *
 * 使用说明：
 * 1. 确保硬件连接正确
 * 2. 上传程序到ESP32
 * 3. 打开串口监视器(115200波特率)
 * 4. 观察电机状态数据
 *
 * 作者：基于小米电机通信协议实现
 * 日期：2024年
 */

#include <Arduino.h>
#include "TWAI_CAN_MI_Motor.h"

// 创建电机控制对象
MI_Motor_ M1_con;

// 控制参数
float target_speed = 0.0;      // 目标速度 (rad/s)
float target_position = 0.0;   // 目标位置 (rad)
float target_current = 0.0;    // 目标电流 (A)
uint8_t current_mode = SPEED_MODE;  // 当前控制模式

// 定时器变量
unsigned long last_update_time = 0;
unsigned long last_control_time = 0;
const unsigned long UPDATE_INTERVAL = 50;   // 数据更新间隔(ms)
const unsigned long CONTROL_INTERVAL = 100; // 控制命令间隔(ms)

// 控制状态
bool motor_enabled = false;
bool system_initialized = false;

void setup() {
    // 初始化串口
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== 小米电机控制程序 ESP32 ===");
    Serial.println("正在初始化系统...");

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

    // 设置为速度模式
    Serial.println("4. 设置速度控制模式...");
    M1_con.Change_Mode(SPEED_MODE);
    delay(200);

    // 使能电机
    Serial.println("5. 使能电机...");
    M1_con.Motor_Enable();
    delay(500);

    // 设置初始速度为0
    M1_con.Set_SpeedMode(0.0);
    delay(200);

    motor_enabled = true;
    system_initialized = true;

    Serial.println("=== 系统初始化完成 ===");
    Serial.println("电机已使能，开始运行...");
    Serial.println();
    Serial.println("实时数据：");
    Serial.println("格式：主站ID,电机ID,错误状态,HALL错误,磁编码错误,温度错误,电流错误,电压错误,模式状态,角度,速度,扭矩,温度");
    Serial.println("----------------------------------------------------------------------------------");

    last_update_time = millis();
    last_control_time = millis();
}

void loop() {
    unsigned long current_time = millis();

    // 定期更新电机数据
    if (current_time - last_update_time >= UPDATE_INTERVAL) {
        last_update_time = current_time;

        // 更新电机数据
        uint8_t result = M1_con.Motor_Data_Updata(20);

        if (result == 0) {
            // 成功读取数据
            print_motor_data();
        } else if (result == 1) {
            Serial.println("警告：接收到其他电机的数据");
        } else {
            Serial.println("警告：未接收到电机数据");
        }
    }

    // 定期发送控制命令
    if (current_time - last_control_time >= CONTROL_INTERVAL) {
        last_control_time = current_time;

        // 执行控制逻辑
        motor_control_logic();
    }

    // 处理串口命令（可选）
    if (Serial.available()) {
        handle_serial_command();
    }

    // 小延时，防止CPU占用过高
    delay(10);
}

void motor_control_logic() {
    static int step = 0;
    static unsigned long step_start_time = 0;
    static float speed_sequence[] = {0.0, 2.0, -2.0, 1.0, -1.0, 0.0};  // 速度序列
    static int speed_count = sizeof(speed_sequence) / sizeof(speed_sequence[0]);

    // 每5秒改变一次速度
    if (millis() - step_start_time > 5000) {
        step_start_time = millis();

        target_speed = speed_sequence[step % speed_count];
        step++;

        Serial.print("设置目标速度：");
        Serial.print(target_speed);
        Serial.println(" rad/s");

        // 发送速度命令
        M1_con.Set_SpeedMode(target_speed);
    }
}

void print_motor_data() {
    can_rx_frame_t* data = &M1_con.motor_rx_data;

    Serial.printf("M1: %d,%d,%d,%d,%d,%d,%d,%d,%d,",
                  data->master_id, data->motor_id,
                  data->err_sta, data->HALL_err,
                  data->magnet_err, data->temp_err,
                  data->current_err, data->voltage_err,
                  data->mode_sta);

    Serial.printf("angle:%.2f,speed:%.2f,torque:%.2f,temp:%.1f\r\n",
                  data->cur_angle, data->cur_speed,
                  data->cur_torque, data->cur_temp);

    // 检查错误状态
    if (data->err_sta) {
        Serial.print("⚠️  电机错误：");
        if (data->HALL_err) Serial.print("HALL ");
        if (data->magnet_err) Serial.print("磁编码器 ");
        if (data->temp_err) Serial.print("温度 ");
        if (data->current_err) Serial.print("电流 ");
        if (data->voltage_err) Serial.print("电压 ");
        Serial.println();
    }

    // 检查温度
    if (data->cur_temp > 60.0) {
        Serial.printf("🌡️  温度警告：%.1f°C\r\n", data->cur_temp);
    }
}

void handle_serial_command() {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "stop") {
        Serial.println("停止电机");
        M1_con.Set_SpeedMode(0.0);
        target_speed = 0.0;

    } else if (command == "enable") {
        Serial.println("使能电机");
        M1_con.Motor_Enable();
        motor_enabled = true;

    } else if (command == "disable") {
        Serial.println("禁用电机");
        M1_con.Motor_Reset();
        motor_enabled = false;

    } else if (command.startsWith("speed")) {
        float speed = command.substring(6).toFloat();
        if (abs(speed) <= 30.0) {
            Serial.printf("设置速度：%.2f rad/s\r\n", speed);
            M1_con.Set_SpeedMode(speed);
            target_speed = speed;
        } else {
            Serial.println("速度超出范围(-30到30 rad/s)");
        }

    } else if (command.startsWith("pos")) {
        float pos = command.substring(4).toFloat();
        if (abs(pos) <= 12.5) {
            Serial.printf("切换到位置模式，设置位置：%.2f rad\r\n", pos);
            M1_con.Change_Mode(POS_MODE);
            delay(100);
            M1_con.Set_PosMode(pos, 5.0);  // 限制最大速度5 rad/s
            current_mode = POS_MODE;
            target_position = pos;
        } else {
            Serial.println("位置超出范围(-12.5到12.5 rad)");
        }

    } else if (command == "speed_mode") {
        Serial.println("切换到速度模式");
        M1_con.Change_Mode(SPEED_MODE);
        delay(100);
        current_mode = SPEED_MODE;

    } else if (command == "help") {
        print_help();

    } else if (command == "zero") {
        Serial.println("设置零点");
        M1_con.Motor_Set_Zero();
        delay(500);

    } else {
        Serial.println("未知命令，输入'help'查看帮助");
    }
}

void print_help() {
    Serial.println("=== 命令帮助 ===");
    Serial.println("stop          - 停止电机");
    Serial.println("enable        - 使能电机");
    Serial.println("disable       - 禁用电机");
    Serial.println("speed X       - 设置速度X rad/s (范围: -30到30)");
    Serial.println("pos X         - 位置模式，设置位置X rad (范围: -12.5到12.5)");
    Serial.println("speed_mode    - 切换到速度模式");
    Serial.println("zero          - 设置当前位置为零点");
    Serial.println("help          - 显示帮助");
    Serial.println();
}

// 程序结束时的清理工作
void cleanup() {
    if (motor_enabled) {
        Serial.println("正在停止电机...");
        M1_con.Motor_Reset();
        delay(100);
    }
}