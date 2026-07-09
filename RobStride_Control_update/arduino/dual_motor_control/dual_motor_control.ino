/*
 * 双电机控制程序 - ESP32版本
 *
 * 控制两个小米电机，支持动态配置电机ID
 * 支持0-360度角度输入，更直观易用！
 * 支持逗号分隔的双电机同步控制
 *
 * 串口命令：
 * - 双电机控制：90,180 (电机1到90度，电机2到180度)
 * - 单电机控制：1-90(电机1转到90度), 2-180(电机2转到180度)
 * - 弧度输入：1.57,3.14 (电机1到π/2弧度，电机2到π弧度)
 * - 配置命令：id1 X(设置电机1ID), id2 X(设置电机2ID), config(显示配置)
 * - 状态命令：status(显示状态), stop(停止), zero(设零点), restart(重启)
 * - 参数调节：kp1 X(电机1位置KP), kp2 X(电机2位置KP), speed X(速度)
 *
 * 使用示例：
 * - 90,360      # 电机1到90度，电机2到360度(回到零点)
 * - 0,0         # 双电机都回到零点
 * - 45,135      # 电机1到45度，电机2到135度
 *
 * 角度范围说明：
 * - 0°    = 正前方/零点位置
 * - 90°   = 顺时针90度
 * - 180°  = 顺时针180度
 * - 270°  = 顺时针270度
 * - 360°  = 回到零点
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "TWAI_CAN_MI_Motor.h"

// 双电机配置
struct DualMotorConfig {
  uint8_t motor1_id;       // 电机1 ID
  uint8_t motor2_id;       // 电机2 ID
  uint8_t master_id;       // 主机ID
  float max_speed;         // 最大速度
  float position_kp1;      // 电机1位置KP
  float position_kp2;      // 电机2位置KP
  float torque_limit;      // 转矩限制
  bool initialized;        // 是否已初始化
};

DualMotorConfig config;
MI_Motor_ motor1;
MI_Motor_ motor2;

// EEPROM地址
#define EEPROM_CONFIG_ADDR 0

// 函数声明
void load_config();
void save_config();
void set_default_config();
void print_config();
void print_motor_status();
void init_motors();
void restart_motors();
void process_dual_command(String cmd);
void set_motor_position(MI_Motor_& motor, float pos, uint8_t motor_num);
void sync_control(float pos1, float pos2);
void set_motor_kp(int motor_num, float kp);
void set_motor_speed(float speed);

void setup() {
  Serial.begin(115200);
  Serial.println("=== 双电机控制程序 ===");
  Serial.println("控制两个小米电机，支持独立和同步控制");
  Serial.println("支持0-360度角度输入和动态ID配置");
  Serial.println();

  // 初始化EEPROM
  EEPROM.begin(512);
  delay(100);

  // 加载或设置默认配置
  load_config();
  print_config();

  // 初始化CAN总线
  Serial.println("初始化CAN总线...");
  Motor_CAN_Init();
  delay(100);

  // 初始化双电机
  init_motors();

  Serial.println("就绪！支持双电机控制");
  Serial.println("双电机控制：90,180 (电机1到90度，电机2到180度)");
  Serial.println("单电机控制：1-90(电机1), 2-180(电机2)");
  Serial.println("示例：90,360 / 0,0 / 45,135");
  Serial.println("配置命令：config(配置), id1 2(电机1ID), id2 3(电机2ID)");
  Serial.println("参数命令：kp1 25(电机1KP), kp2 30(电机2KP), speed 3.0(速度)");
  Serial.println("状态命令：status(状态), stop(停止), zero(设零点), restart(重启)");
}

void loop() {
  // 更新电机状态
  motor1.Motor_Data_Updata(50);
  motor2.Motor_Data_Updata(50);

  // 显示当前位置（每2秒）
  static unsigned long last_print = 0;
  if (millis() - last_print > 2000) {
    last_print = millis();

    float pos1 = motor1.motor_rx_data.cur_angle;
    float deg1 = pos1 * 180.0 / PI;
    float speed1 = motor1.motor_rx_data.cur_speed;

    float pos2 = motor2.motor_rx_data.cur_angle;
    float deg2 = pos2 * 180.0 / PI;
    float speed2 = motor2.motor_rx_data.cur_speed;

    // 归一化角度到0-360度范围
    float display_angle1 = deg1;
    while (display_angle1 < 0) display_angle1 += 360.0;
    while (display_angle1 >= 360.0) display_angle1 -= 360.0;

    float display_angle2 = deg2;
    while (display_angle2 < 0) display_angle2 += 360.0;
    while (display_angle2 >= 360.0) display_angle2 -= 360.0;

    Serial.printf("电机1(ID=%d): 位置 %.3f rad (%.1f°), 速度: %.2f rad/s\r\n",
                  config.motor1_id, pos1, display_angle1, speed1);
    Serial.printf("电机2(ID=%d): 位置 %.3f rad (%.1f°), 速度: %.2f rad/s\r\n",
                  config.motor2_id, pos2, display_angle2, speed2);
    Serial.println();
  }

  // 处理串口命令
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    process_dual_command(cmd);
  }

  delay(20);
}

void process_dual_command(String cmd) {
  // 基本控制命令
  if (cmd == "stop") {
    float pos1 = motor1.motor_rx_data.cur_angle;
    float pos2 = motor2.motor_rx_data.cur_angle;
    motor1.Set_SingleParameter(0x7016, pos1);
    motor2.Set_SingleParameter(0x7016, pos2);
    Serial.println("双电机停止");

  } else if (cmd == "zero") {
    motor1.Motor_Set_Zero();
    motor2.Motor_Set_Zero();
    delay(300);
    Serial.println("双电机设零点完成");

  } else if (cmd == "config") {
    print_config();

  } else if (cmd == "status") {
    print_motor_status();

  } else if (cmd == "restart") {
    restart_motors();

  }
  // 双电机控制：90,180 (逗号分隔)
  else if (cmd.indexOf(",") != -1) {
    int comma_pos = cmd.indexOf(",");
    String angle1_str = cmd.substring(0, comma_pos);
    String angle2_str = cmd.substring(comma_pos + 1);

    angle1_str.trim();
    angle2_str.trim();

    if (angle1_str.length() > 0 && angle2_str.length() > 0) {
      float angle1 = angle1_str.toFloat();
      float angle2 = angle2_str.toFloat();

      if ((angle1 != 0 || angle1_str == "0") && (angle2 != 0 || angle2_str == "0")) {
        sync_control(angle1, angle2);
      } else {
        Serial.println("❌ 角度值无效，应为: <角度1>,<角度2>");
      }
    } else {
      Serial.println("❌ 双电机控制格式错误，应为: <角度1>,<角度2>");
    }
  }
  // 单电机控制：1-90 (电机1到90度)
  else if (cmd.startsWith("1-") || cmd.startsWith("2-")) {
    char motor_char = cmd.charAt(0);
    int motor_num = motor_char - '0';
    String value_str = cmd.substring(2);

    if (value_str.length() > 0) {
      float value = value_str.toFloat();
      if (value != 0 || value_str == "0") {
        MI_Motor_& motor = (motor_num == 1) ? motor1 : motor2;
        set_motor_position(motor, value, motor_num);
      }
    }

  }
  // 保留旧的sync命令格式作为兼容
  else if (cmd.startsWith("sync ")) {
    String params = cmd.substring(5);
    int space_pos = params.indexOf(' ');

    if (space_pos != -1) {
      float pos1 = params.substring(0, space_pos).toFloat();
      float pos2 = params.substring(space_pos + 1).toFloat();
      sync_control(pos1, pos2);
    } else {
      Serial.println("❌ 同步控制格式错误，应为: sync <角度1> <角度2>");
    }

  }
  // 配置命令
  else if (cmd.startsWith("id1 ")) {
    String id_str = cmd.substring(4);
    int new_id = id_str.toInt();
    if (new_id >= 1 && new_id <= 127) {
      config.motor1_id = (uint8_t)new_id;
      save_config();
      Serial.printf("电机1 ID已设置为: %d\r\n", new_id);
      Serial.println("请输入 'restart' 重启电机以应用新ID");
    } else {
      Serial.println("❌ 电机ID范围应为1-127");
    }

  } else if (cmd.startsWith("id2 ")) {
    String id_str = cmd.substring(4);
    int new_id = id_str.toInt();
    if (new_id >= 1 && new_id <= 127) {
      config.motor2_id = (uint8_t)new_id;
      save_config();
      Serial.printf("电机2 ID已设置为: %d\r\n", new_id);
      Serial.println("请输入 'restart' 重启电机以应用新ID");
    } else {
      Serial.println("❌ 电机ID范围应为1-127");
    }

  }
  // 参数调节命令
  else if (cmd.startsWith("kp1 ")) {
    String kp_str = cmd.substring(4);
    float new_kp = kp_str.toFloat();
    if (new_kp > 0 && new_kp <= 200) {
      set_motor_kp(1, new_kp);
    } else {
      Serial.println("❌ KP范围应为0-200");
    }

  } else if (cmd.startsWith("kp2 ")) {
    String kp_str = cmd.substring(4);
    float new_kp = kp_str.toFloat();
    if (new_kp > 0 && new_kp <= 200) {
      set_motor_kp(2, new_kp);
    } else {
      Serial.println("❌ KP范围应为0-200");
    }

  } else if (cmd.startsWith("speed ")) {
    String speed_str = cmd.substring(6);
    float new_speed = speed_str.toFloat();
    if (new_speed > 0 && new_speed <= 200) {
      set_motor_speed(new_speed);
    } else {
      Serial.println("❌ 速度范围应为0-200 rad/s");
    }

  } else if (cmd == "help") {
    Serial.println("=== 双电机控制命令 ===");
    Serial.println("双电机控制 (推荐):");
    Serial.println("  90,180      # 电机1到90度，电机2到180度");
    Serial.println("  0,0         # 双电机都回到0度");
    Serial.println("  90,360      # 电机1到90度，电机2到360度(回零)");
    Serial.println("  45,135      # 电机1到45度，电机2到135度");
    Serial.println("  1.57,3.14   # 弧度输入：电机1到π/2，电机2到π");
    Serial.println();
    Serial.println("单电机控制:");
    Serial.println("  1-90        # 电机1转到90度");
    Serial.println("  2-180       # 电机2转到180度");
    Serial.println("  1-0         # 电机1回到0度");
    Serial.println("  2-270       # 电机2转到270度");
    Serial.println();
    Serial.println("配置命令:");
    Serial.println("  id1 2       # 设置电机1ID为2");
    Serial.println("  id2 3       # 设置电机2ID为3");
    Serial.println("  config      # 显示当前配置");
    Serial.println();
    Serial.println("参数命令:");
    Serial.println("  kp1 25      # 设置电机1位置KP");
    Serial.println("  kp2 30      # 设置电机2位置KP");
    Serial.println("  speed 3.0   # 设置最大速度");
    Serial.println();
    Serial.println("控制命令:");
    Serial.println("  stop        # 停止双电机");
    Serial.println("  zero        # 设零点");
    Serial.println("  restart     # 重启双电机");
    Serial.println("  status      # 显示状态");
    Serial.println("========================");

  } else {
    Serial.println("❌ 未知命令，输入 'help' 查看帮助");
  }
}

void set_motor_position(MI_Motor_& motor, float value, uint8_t motor_num) {
  float target_pos;
  bool is_angle = false;

  // 智能判断是弧度还是角度
  if (value >= 0 && value <= 360.0) {
    // 0-360范围内，优先识别为角度
    is_angle = true;
    target_pos = (value - 180.0) * PI / 180.0;
    Serial.printf("电机%d 角度 %.1f° -> 弧度 %.3f\r\n", motor_num, value, target_pos);
  } else if (abs(value) <= 6.28) {
    // 小弧度值，识别为弧度
    target_pos = value;
    Serial.printf("电机%d 弧度位置: %.3f\r\n", motor_num, target_pos);
  } else if (value > 360.0) {
    // 大于360度，按角度处理但给出警告
    float normalized_angle = fmod(value, 360.0);
    is_angle = true;
    target_pos = (normalized_angle - 180.0) * PI / 180.0;
    Serial.printf("电机%d 角度 %.1f° -> 归一化 %.1f° -> 弧度 %.3f\r\n",
                  motor_num, value, normalized_angle, target_pos);
  } else {
    // 其他情况，按弧度处理
    target_pos = value;
    Serial.printf("电机%d 弧度位置: %.3f\r\n", motor_num, target_pos);
  }

  // 限制范围（小米电机范围：-12.5到12.5弧度）
  if (target_pos > 12.0) {
    target_pos = 12.0;
    Serial.printf("电机%d ⚠️ 超出范围，限制到最大值\r\n", motor_num);
  }
  if (target_pos < -12.0) {
    target_pos = -12.0;
    Serial.printf("电机%d ⚠️ 超出范围，限制到最小值\r\n", motor_num);
  }

  // 设置位置
  motor.Set_SingleParameter(0x7016, target_pos);

  if (is_angle) {
    float actual_deg = target_pos * 180.0 / PI;
    Serial.printf("电机%d 设置位置: %.3f rad (%.1f°)\r\n", motor_num, target_pos, actual_deg);
  } else {
    Serial.printf("电机%d 设置位置: %.3f rad\r\n", motor_num, target_pos);
  }
}

void sync_control(float pos1, float pos2) {
  Serial.printf("双电机控制: 电机1->%.1f°, 电机2->%.1f°\r\n", pos1, pos2);

  set_motor_position(motor1, pos1, 1);
  delay(50);
  set_motor_position(motor2, pos2, 2);

  Serial.println("✓ 双电机控制命令已发送");
}

// ========== 配置相关函数 ==========

void load_config() {
  EEPROM.get(EEPROM_CONFIG_ADDR, config);

  if (!config.initialized || config.motor1_id == 0 || config.motor1_id > 127 ||
      config.motor2_id == 0 || config.motor2_id > 127) {
    Serial.println("首次运行或配置无效，使用默认配置");
    set_default_config();
    save_config();
  } else {
    Serial.println("已加载保存的配置");
  }
}

void save_config() {
  EEPROM.put(EEPROM_CONFIG_ADDR, config);
  EEPROM.commit();
  Serial.println("配置已保存");
}

void set_default_config() {
  config.motor1_id = 1;        // 默认电机1 ID
  config.motor2_id = 2;        // 默认电机2 ID
  config.master_id = 0;       // 默认主机ID
  config.max_speed = 2.0;     // 默认最大速度
  config.position_kp1 = 30.0; // 默认电机1位置KP
  config.position_kp2 = 30.0; // 默认电机2位置KP
  config.torque_limit = 12.0; // 默认转矩限制
  config.initialized = true;   // 标记为已初始化
}

void print_config() {
  Serial.println("=== 双电机配置 ===");
  Serial.printf("电机1 ID: %d\r\n", config.motor1_id);
  Serial.printf("电机2 ID: %d\r\n", config.motor2_id);
  Serial.printf("主机ID: %d\r\n", config.master_id);
  Serial.printf("最大速度: %.1f rad/s\r\n", config.max_speed);
  Serial.printf("电机1位置KP: %.1f\r\n", config.position_kp1);
  Serial.printf("电机2位置KP: %.1f\r\n", config.position_kp2);
  Serial.printf("转矩限制: %.1f Nm\r\n", config.torque_limit);
  Serial.println("==================");
}

void init_motors() {
  Serial.printf("初始化双电机 ID=%d, %d...\r\n", config.motor1_id, config.motor2_id);

  // 初始化电机1
  motor1.Motor_Con_Init(config.motor1_id);
  delay(100);
  motor1.Motor_Set_Zero();
  delay(300);
  motor1.Change_Mode(POS_MODE);
  delay(100);
  motor1.Set_SingleParameter(0x7017, config.max_speed);
  delay(50);
  motor1.Set_SingleParameter(0x701E, config.position_kp1);
  delay(50);
  motor1.Motor_Enable();
  delay(300);

  // 初始化电机2
  motor2.Motor_Con_Init(config.motor2_id);
  delay(100);
  motor2.Motor_Set_Zero();
  delay(300);
  motor2.Change_Mode(POS_MODE);
  delay(100);
  motor2.Set_SingleParameter(0x7017, config.max_speed);
  delay(50);
  motor2.Set_SingleParameter(0x701E, config.position_kp2);
  delay(50);
  motor2.Motor_Enable();
  delay(300);

  Serial.println("双电机初始化完成！");
}

void restart_motors() {
  Serial.println("重启双电机...");

  // 先停止当前电机
  motor1.Motor_Reset();
  motor2.Motor_Reset();
  delay(200);

  // 重新初始化
  init_motors();

  Serial.println("双电机重启完成！");
}

void print_motor_status() {
  Serial.println("=== 双电机状态 ===");

  // 电机1状态
  float pos1 = motor1.motor_rx_data.cur_angle;
  float deg1 = pos1 * 180.0 / PI;
  float speed1 = motor1.motor_rx_data.cur_speed;
  float display_angle1 = deg1;
  while (display_angle1 < 0) display_angle1 += 360.0;
  while (display_angle1 >= 360.0) display_angle1 -= 360.0;

  // 电机2状态
  float pos2 = motor2.motor_rx_data.cur_angle;
  float deg2 = pos2 * 180.0 / PI;
  float speed2 = motor2.motor_rx_data.cur_speed;
  float display_angle2 = deg2;
  while (display_angle2 < 0) display_angle2 += 360.0;
  while (display_angle2 >= 360.0) display_angle2 -= 360.0;

  Serial.printf("电机1(ID=%d):\r\n", config.motor1_id);
  Serial.printf("  位置: %.3f rad (%.1f°)\r\n", pos1, display_angle1);
  Serial.printf("  速度: %.3f rad/s (%.1f RPM)\r\n", speed1, speed1 * 60.0 / (2.0 * PI));

  Serial.printf("电机2(ID=%d):\r\n", config.motor2_id);
  Serial.printf("  位置: %.3f rad (%.1f°)\r\n", pos2, display_angle2);
  Serial.printf("  速度: %.3f rad/s (%.1f RPM)\r\n", speed2, speed2 * 60.0 / (2.0 * PI));

  Serial.println("==================");
}

void set_motor_kp(int motor_num, float kp) {
  if (motor_num == 1) {
    motor1.Set_SingleParameter(0x701E, kp);
    config.position_kp1 = kp;
    Serial.printf("✓ 电机1位置KP已设置为: %.1f\r\n", kp);
  } else if (motor_num == 2) {
    motor2.Set_SingleParameter(0x701E, kp);
    config.position_kp2 = kp;
    Serial.printf("✓ 电机2位置KP已设置为: %.1f\r\n", kp);
  }
  save_config();
  Serial.println("提示：新的KP值已保存到配置中");
}

void set_motor_speed(float speed) {
  motor1.Set_SingleParameter(0x7017, speed);
  motor2.Set_SingleParameter(0x7017, speed);
  config.max_speed = speed;
  save_config();
  Serial.printf("✓ 双电机最大速度已设置为: %.1f rad/s\r\n", speed);
  Serial.println("提示：新的速度限制已保存到配置中");
}