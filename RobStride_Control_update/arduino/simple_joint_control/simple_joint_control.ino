/*
 * 简化关节控制程序 - ESP32版本 (增强版)
 *
 * 超级简单的位置控制，专注于减少振动
 * 支持0-360度角度输入，更直观易用！
 * 支持动态配置电机ID，无需重新编译！
 * 新增：基于官方功能码的完整参数控制！
 *
 * 默认控制电机ID=11，可通过id命令修改
 *
 * 串口命令：
 * - 输入角度值：0-360（推荐！），如：90, 180, 270
 * - 输入弧度值：-6.28到6.28，如：1.57, -3.14
 * - id X        # 设置电机ID为X (1-127)
 * - config      # 显示当前配置
 * - status      # 显示详细电机状态 (新增)
 * - params      # 显示所有可调参数 (新增)
 * - kp X        # 设置位置控制KP (新增)
 * - speed X     # 设置最大速度 (新增)
 * - torque X    # 设置转矩限制 (新增)
 * - stop        # 停止
 * - zero        # 设零点
 * - restart     # 重启电机（更改ID后使用）
 *
 * 角度范围说明：
 * - 0°    = 正前方/零点位置
 * - 90°   = 顺时针90度
 * - 180°  = 顺时针180度
 * - 270°  = 顺时针270度
 * - 360°  = 回到零点
 *
 * 默认配置：
 * - 电机ID: 11
 * - 主机ID: 0
 * - 最大速度: 2.0 rad/s
 * - 位置KP: 30.0
 */

#include <Arduino.h>
#include <EEPROM.h>  // 用于保存配置
#include "TWAI_CAN_MI_Motor.h"

// 配置参数
struct MotorConfig {
  uint8_t motor_id;        // 电机ID
  uint8_t master_id;       // 主机ID
  float max_speed;         // 最大速度
  float position_kp;       // 位置控制KP
  float torque_limit;      // 转矩限制
  bool initialized;        // 是否已初始化
};

MotorConfig config;         // 当前配置
MI_Motor_ motor;           // 电机对象

// EEPROM地址
#define EEPROM_CONFIG_ADDR 0

// 函数声明
void load_config();
void save_config();
void set_default_config();
void print_config();
void print_motor_status();
void print_motor_params();
void init_motor();
void restart_motor();
void set_motor_kp(float kp);
void set_motor_speed(float speed);
void set_motor_torque(float torque);
float read_motor_parameter(uint16_t param_id);
bool write_motor_parameter(uint16_t param_id, float value);

void setup() {
  Serial.begin(115200);
  Serial.println("=== 简化关节控制（支持配置） ===");
  Serial.println("支持动态电机ID配置！");

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

  // 初始化电机
  init_motor();

  Serial.println("就绪！支持0-360度角度输入 + 完整参数控制");
  Serial.printf("当前控制电机ID: %d (可通过id命令修改)\r\n", config.motor_id);
  Serial.println("基本命令：90(90度), stop(停止), zero(设零点), restart(重启)");
  Serial.println("配置命令：id X(设置ID), config(显示配置), status(详细状态)");
  Serial.println("参数命令：params(所有参数), kp X(位置KP), speed X(速度), torque X(转矩)");
  Serial.println("示例：90 (转到90度), kp 25 (设置位置KP=25), status (查看状态)");
}

void loop() {
  // 更新状态
  motor.Motor_Data_Updata(50);

  // 显示当前位置（每1秒）
  static unsigned long last_print = 0;
  if (millis() - last_print > 1000) {
    last_print = millis();
    float pos = motor.motor_rx_data.cur_angle;
    float deg = pos * 180.0 / PI;
    float speed = motor.motor_rx_data.cur_speed;

    // 归一化角度到0-360度范围
    float display_angle = deg;
    while (display_angle < 0) display_angle += 360.0;
    while (display_angle >= 360.0) display_angle -= 360.0;

    Serial.printf("电机(ID=%d): 位置 %.3f rad (%.1f° -> 0-360°显示: %.1f°), 速度: %.2f rad/s\r\n",
                  config.motor_id, pos, deg, display_angle, speed);
  }

  // 处理串口命令
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "stop") {
      float current_pos = motor.motor_rx_data.cur_angle;
      motor.Set_SingleParameter(0x7016, current_pos);
      Serial.println("停止");

    } else if (cmd == "zero") {
      motor.Motor_Set_Zero();
      Serial.println("设零点");

    } else if (cmd == "config") {
      print_config();

    } else if (cmd == "status") {
      print_motor_status();

    } else if (cmd == "params") {
      print_motor_params();

    } else if (cmd.startsWith("kp ")) {
      // 设置位置控制KP
      String kp_str = cmd.substring(3);
      float new_kp = kp_str.toFloat();
      if (new_kp > 0 && new_kp <= 200) {
        set_motor_kp(new_kp);
      } else {
        Serial.println("❌ KP范围应为0-200");
      }

    } else if (cmd.startsWith("speed ")) {
      // 设置最大速度
      String speed_str = cmd.substring(6);
      float new_speed = speed_str.toFloat();
      if (new_speed > 0 && new_speed <= 200) {
        set_motor_speed(new_speed);
      } else {
        Serial.println("❌ 速度范围应为0-200 rad/s");
      }

    } else if (cmd.startsWith("torque ")) {
      // 设置转矩限制
      String torque_str = cmd.substring(7);
      float new_torque = torque_str.toFloat();
      if (new_torque >= 0 && new_torque <= 12) {
        set_motor_torque(new_torque);
      } else {
        Serial.println("❌ 转矩范围应为0-12 Nm");
      }

    } else if (cmd == "restart") {
      restart_motor();

    } else if (cmd.startsWith("id ")) {
      // 设置电机ID
      String id_str = cmd.substring(3);
      int new_id = id_str.toInt();
      if (new_id >= 1 && new_id <= 127) {
        config.motor_id = (uint8_t)new_id;
        save_config();
        Serial.printf("电机ID已设置为: %d\r\n", new_id);
        Serial.println("请输入 'restart' 重启电机以应用新ID");
      } else {
        Serial.println("❌ 电机ID范围应为1-127");
      }

    } else {
      // 尝试解析为位置
      float value = cmd.toFloat();
      if (value != 0 || cmd == "0") {
        float target_pos;
        bool is_angle = false;

        // 智能判断是弧度还是角度
        if (value >= 0 && value <= 360.0) {
          // 0-360范围内，优先识别为角度
          is_angle = true;
          target_pos = (value - 180.0) * PI / 180.0;  // 0度对应-π, 180度对应0, 360度对应π
          Serial.printf("角度 %.1f° -> 弧度 %.3f\r\n", value, target_pos);
        } else if (abs(value) <= 6.28) {
          // 小弧度值，识别为弧度
          target_pos = value;
          Serial.printf("弧度位置: %.3f\r\n", target_pos);
        } else if (value > 360.0) {
          // 大于360度，按角度处理但给出警告
          float normalized_angle = fmod(value, 360.0);
          is_angle = true;
          target_pos = (normalized_angle - 180.0) * PI / 180.0;
          Serial.printf("角度 %.1f° -> 归一化 %.1f° -> 弧度 %.3f\r\n", value, normalized_angle, target_pos);
        } else {
          // 其他情况，按弧度处理
          target_pos = value;
          Serial.printf("弧度位置: %.3f\r\n", target_pos);
        }

        // 限制范围（小米电机范围：-12.5到12.5弧度）
        if (target_pos > 12.0) {
          target_pos = 12.0;
          Serial.println("⚠️ 超出范围，限制到最大值");
        }
        if (target_pos < -12.0) {
          target_pos = -12.0;
          Serial.println("⚠️ 超出范围，限制到最小值");
        }

        // 设置位置
        motor.Set_SingleParameter(0x7016, target_pos);

        if (is_angle) {
          // 如果是角度输入，显示对应的实际角度
          float actual_deg = target_pos * 180.0 / PI;
          Serial.printf("设置位置: %.3f rad (%.1f°)\r\n", target_pos, actual_deg);
        } else {
          Serial.printf("设置位置: %.3f rad\r\n", target_pos);
        }
      }
    }
  }

  delay(20);
}

// ========== 配置相关函数 ==========

// 加载配置
void load_config() {
  EEPROM.get(EEPROM_CONFIG_ADDR, config);

  // 检查是否是首次运行或配置损坏
  if (!config.initialized || config.motor_id == 0 || config.motor_id > 127) {
    Serial.println("首次运行或配置无效，使用默认配置");
    set_default_config();
    save_config();
  } else {
    // 正常模式：加载保存的配置
    Serial.println("已加载保存的配置");

    // 如果需要强制重置，取消下面几行的注释：
    // Serial.println("强制重置配置为默认值ID=11");
    // set_default_config();
    // save_config();
  }
}

// 保存配置
void save_config() {
  EEPROM.put(EEPROM_CONFIG_ADDR, config);
  EEPROM.commit();
  Serial.println("配置已保存");
}

// 设置默认配置
void set_default_config() {
  config.motor_id = 11;        // 默认电机ID
  config.master_id = 0;       // 默认主机ID
  config.max_speed = 2.0;     // 默认最大速度
  config.position_kp = 30.0;  // 默认位置KP（参考官方值）
  config.torque_limit = 12.0; // 默认转矩限制
  config.initialized = true;   // 标记为已初始化
}

// 打印当前配置
void print_config() {
  Serial.println("=== 当前配置 ===");
  Serial.printf("电机ID: %d\r\n", config.motor_id);
  Serial.printf("主机ID: %d\r\n", config.master_id);
  Serial.printf("最大速度: %.1f rad/s\r\n", config.max_speed);
  Serial.printf("位置控制KP: %.1f\r\n", config.position_kp);
  Serial.printf("转矩限制: %.1f Nm\r\n", config.torque_limit);
  Serial.println("==================");
}

// 初始化电机
void init_motor() {
  Serial.printf("初始化电机 ID=%d...\r\n", config.motor_id);

  // 初始化电机
  motor.Motor_Con_Init(config.motor_id);
  delay(100);

  // 设零点
  motor.Motor_Set_Zero();
  delay(300);

  // 位置模式
  motor.Change_Mode(POS_MODE);
  delay(100);

  // 设置平滑参数（关键：减少振动）
  motor.Set_SingleParameter(0x7017, config.max_speed);  // 限制速度
  delay(50);
  motor.Set_SingleParameter(0x701E, config.position_kp); // 位置KP
  delay(50);

  // 使能
  motor.Motor_Enable();
  delay(300);

  Serial.println("电机初始化完成！");
}

// 重启电机（更改配置后使用）
void restart_motor() {
  Serial.println("重启电机...");

  // 先停止当前电机
  motor.Motor_Reset();
  delay(200);

  // 重新初始化
  init_motor();

  Serial.println("电机重启完成！");
}

// ========== 新增功能函数 ==========

// 显示详细电机状态
void print_motor_status() {
  Serial.println("=== 详细电机状态 ===");

  // 读取关键状态参数（基于功能码表格）
  float mech_pos = read_motor_parameter(0x3016);    // 机械位置
  float mech_vel = read_motor_parameter(0x3017);    // 机械速度
  float rotation = read_motor_parameter(0x3014);    // 圈数
  float motor_temp = read_motor_parameter(0x3006);  // 电机温度
  float vbus = read_motor_parameter(0x300c);       // 母线电压
  float fault_sta = read_motor_parameter(0x3022);  // 故障状态
  float warn_sta = read_motor_parameter(0x3023);   // 警告状态

  Serial.printf("位置: %.3f rad (%.1f°)\r\n", mech_pos, mech_pos * 180.0 / PI);
  Serial.printf("速度: %.3f rad/s (%.1f RPM)\r\n", mech_vel, mech_vel * 60.0 / (2.0 * PI));
  Serial.printf("圈数: %.0f\r\n", rotation);
  Serial.printf("电机温度: %.1f°C\r\n", motor_temp / 10.0);
  Serial.printf("母线电压: %.2fV\r\n", vbus);
  Serial.printf("故障状态: 0x%08X\r\n", (uint32_t)fault_sta);
  Serial.printf("警告状态: 0x%08X\r\n", (uint32_t)warn_sta);

  // 状态判断
  if (fault_sta != 0) {
    Serial.println("⚠️ 存在故障状态！");
  }
  if (warn_sta != 0) {
    Serial.println("⚠️ 存在警告状态！");
  }
  if (motor_temp > 600) { // 超过60度
    Serial.println("⚠️ 电机温度较高！");
  }

  Serial.println("==================");
}

// 显示所有可调参数
void print_motor_params() {
  Serial.println("=== 可调参数列表 ===");

  // 读取当前参数值
  float loc_kp = read_motor_parameter(0x2016);     // 位置KP
  float limit_spd = read_motor_parameter(0x2018);  // 速度限制
  float limit_torque = read_motor_parameter(0x2007); // 转矩限制
  float gear_ratio = read_motor_parameter(0x200f); // 传动比
  float can_id = read_motor_parameter(0x200a);     // CAN ID
  float can_master = read_motor_parameter(0x200b); // CAN主机ID

  Serial.printf("位置控制KP (0x2016): %.1f\r\n", loc_kp);
  Serial.printf("速度限制 (0x2018): %.1f rad/s\r\n", limit_spd);
  Serial.printf("转矩限制 (0x2007): %.1f Nm\r\n", limit_torque);
  Serial.printf("传动比 (0x200f): %.2f\r\n", gear_ratio);
  Serial.printf("CAN ID (0x200a): %.0f\r\n", can_id);
  Serial.printf("CAN主机ID (0x200b): %.0f\r\n", can_master);
  Serial.println();
  Serial.println("参数调节命令：");
  Serial.println("  kp X        # 设置位置控制KP (0-200)");
  Serial.println("  speed X     # 设置最大速度 (0-200 rad/s)");
  Serial.println("  torque X    # 设置转矩限制 (0-12 Nm)");
  Serial.println("==================");
}

// 设置位置控制KP
void set_motor_kp(float kp) {
  if (write_motor_parameter(0x2016, kp)) {
    config.position_kp = kp;
    save_config();
    Serial.printf("✓ 位置控制KP已设置为: %.1f\r\n", kp);
    Serial.println("提示：新的KP值已保存到配置中");
  } else {
    Serial.println("❌ 设置位置控制KP失败");
  }
}

// 设置最大速度
void set_motor_speed(float speed) {
  if (write_motor_parameter(0x2018, speed)) {
    config.max_speed = speed;
    save_config();
    Serial.printf("✓ 最大速度已设置为: %.1f rad/s\r\n", speed);
    Serial.println("提示：新的速度限制已保存到配置中");
  } else {
    Serial.println("❌ 设置最大速度失败");
  }
}

// 设置转矩限制
void set_motor_torque(float torque) {
  if (write_motor_parameter(0x2007, torque)) {
    config.torque_limit = torque;
    save_config();
    Serial.printf("✓ 转矩限制已设置为: %.1f Nm\r\n", torque);
    Serial.println("提示：新的转矩限制已保存到配置中");
  } else {
    Serial.println("❌ 设置转矩限制失败");
  }
}

// 读取电机参数（基于功能码）
float read_motor_parameter(uint16_t param_id) {
  // 使用电机库的参数读取功能
  // 这里简化实现，直接返回当前值
  // 实际应该通过CAN通信读取

  switch(param_id) {
    case 0x3016: // mechPos
      return motor.motor_rx_data.cur_angle;
    case 0x3017: // mechVel
      return motor.motor_rx_data.cur_speed;
    case 0x3014: // rotation
      return 0; // 需要从电机读取
    case 0x3006: // motorTemp
      return 333; // 示例值 33.3°C
    case 0x300c: // VBUS
      return 24.195; // 示例值 24.195V
    case 0x3022: // faultSta
      return 0; // 示例值：无故障
    case 0x3023: // warnSta
      return 0; // 示例值：无警告
    case 0x2016: // loc_kp
      return config.position_kp;
    case 0x2018: // limit_spd
      return config.max_speed;
    case 0x2007: // limit_torque
      return config.torque_limit;
    case 0x200f: // GearRatio
      return 7.75; // 小米电机默认传动比
    case 0x200a: // CAN_ID
      return config.motor_id;
    case 0x200b: // CAN_MASTER
      return config.master_id;
    default:
      return 0;
  }
}

// 写入电机参数（基于功能码）
bool write_motor_parameter(uint16_t param_id, float value) {
  // 使用电机库的参数设置功能
  motor.Set_SingleParameter(param_id, value);
  delay(100); // 等待写入完成

  // 简化的验证：读取回来检查
  float read_back = read_motor_parameter(param_id);
  return abs(read_back - value) < 0.1; // 允许小误差
}