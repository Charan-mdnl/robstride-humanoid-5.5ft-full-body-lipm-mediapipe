/*
 * 小米电机ID设置程序 - ESP32版本
 *
 * 通过CAN总线动态修改电机ID，实现多个电机串联控制
 *
 * 功能：
 * - 扫描总线上所有电机
 * - 修改指定电机的ID
 * - 批量设置多个电机ID
 * - 验证ID修改结果
 *
 * 串口命令：
 * - scan                # 扫描总线上的所有电机
 * - set <old_id> <new_id> # 将电机old_id的ID改为new_id
 * - batch <start_id>     # 批量设置从start_id开始的连续ID
 * - verify              # 验证当前ID设置
 * - reset <id>          # 重置指定电机
 * - help                # 显示帮助
 *
 * 使用流程：
 * 1. 先连接单个电机到总线
 * 2. 使用scan扫描当前电机ID
 * 3. 使用set命令修改电机ID
 * 4. 连接下一个电机，重复步骤2-3
 * 5. 使用verify验证所有电机ID设置
 *
 * 注意：修改ID时，总线上只能连接一个电机！
 */

#include <Arduino.h>
#include "TWAI_CAN_MI_Motor.h"

#define CAN_TX          5
#define CAN_RX          4
#define MASTER_ID       0

// 最大支持的电机数量
#define MAX_MOTORS      32

// 发现的电机列表
uint8_t discovered_motors[MAX_MOTORS];
uint8_t motor_count = 0;

// 扫描进度
uint8_t scan_start_id = 1;
uint8_t scan_end_id = 127;

// 调试模式
bool debug_mode = false;

// 函数声明
void process_command(String command);
void process_set_command(String command);
void process_batch_command(String command);
void process_scan_range_command(String command);
void process_reset_command(String command);
void scan_motors();
bool ping_motor(uint8_t motor_id);
void clear_can_buffer();

void setup() {
  Serial.begin(115200);
  Serial.println("=== 小米电机ID设置程序 ===");
  Serial.println("通过CAN总线修改电机ID，支持多电机串联控制");
  Serial.println();

  // 初始化CAN总线
  Serial.println("初始化CAN总线...");
  Motor_CAN_Init();
  delay(200);

  Serial.println("系统初始化完成！");
  Serial.println();
  print_help();
  Serial.println("准备就绪，请输入命令...");
}

void loop() {
  // 处理串口命令
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      process_command(command);
    }
  }

  delay(50);
}

void process_command(String command) {
  Serial.print("执行命令: ");
  Serial.println(command);

  if (command == "help" || command == "h") {
    print_help();
  } else if (command == "test") {
    test_can_communication();
  } else if (command.startsWith("test ")) {
    // 测试指定ID的电机
    String id_str = command.substring(5);
    int test_id = id_str.toInt();
    if (test_id >= 1 && test_id <= 127) {
      Serial.printf("测试电机ID=%d...\r\n", test_id);
      if (ping_motor(test_id)) {
        Serial.printf("✅ 电机ID=%d响应正常！\r\n", test_id);
      } else {
        Serial.printf("❌ 电机ID=%d无响应\r\n", test_id);
      }
    } else {
      Serial.println("❌ ID范围错误，应为1-127");
    }
  } else if (command == "debug") {
    debug_mode = true;
    Serial.println("调试模式已开启，将显示详细CAN通信信息");
  } else if (command == "nodebug") {
    debug_mode = false;
    Serial.println("调试模式已关闭");
  } else if (command == "scan" || command == "s") {
    scan_motors();
  } else if (command == "verify" || command == "v") {
    verify_motors();
  } else if (command.startsWith("set ")) {
    process_set_command(command);
  } else if (command.startsWith("batch ")) {
    process_batch_command(command);
  } else if (command.startsWith("reset ")) {
    process_reset_command(command);
  } else if (command.startsWith("scan ")) {
    process_scan_range_command(command);
  } else {
    Serial.println("❌ 未知命令，输入 'help' 查看帮助");
  }

  Serial.println();
}

void print_help() {
  Serial.println("=== 命令帮助 ===");
  Serial.println("基础命令:");
  Serial.println("  help/h               # 显示帮助");
  Serial.println("  test                 # 测试CAN通信");
  Serial.println("  scan                 # 扫描总线上的电机(1-127)");
  Serial.println("  scan 1 10           # 扫描指定范围的电机ID");
  Serial.println("  verify               # 验证发现的电机ID");
  Serial.println();
  Serial.println("ID设置命令 (⚠️  修改ID时总线上只能连接一个电机！):");
  Serial.println("  set 1 2             # 将ID=1的电机改为ID=2");
  Serial.println("  batch 5             # 批量设置连续ID(1,2,3...)");
  Serial.println("  reset 3             # 重置指定电机");
  Serial.println("  test 11             # 测试指定ID的电机");
  Serial.println();
  Serial.println("调试命令:");
  Serial.println("  debug                # 开启调试模式");
  Serial.println("  nodebug              # 关闭调试模式");
  Serial.println();
  Serial.println("示例:");
  Serial.println("  test                 # 测试CAN通信");
  Serial.println("  scan                 # 扫描所有电机");
  Serial.println("  set 1 2             # 将ID=1的电机改为ID=2");
  Serial.println("  batch 3             # 批量设置ID: 1,2,3");
  Serial.println("  verify              # 验证设置结果");
  Serial.println();
  Serial.println("⚠️  重要提示:");
  Serial.println("1. 修改ID时，总线上只能连接一个电机");
  Serial.println("2. 修改完成后，断开电机，连接下一个，继续修改");
  Serial.println("3. 所有电机修改完成后，可以全部连接到总线");
  Serial.println("4. ID范围: 1-127");
  Serial.println("5. 如果扫描失败，先运行'test'测试通信");
  Serial.println("========================");
}

void scan_motors() {
  Serial.printf("扫描电机ID %d-%d...\r\n", scan_start_id, scan_end_id);
  motor_count = 0;

  // 清空电机列表
  for (int i = 0; i < MAX_MOTORS; i++) {
    discovered_motors[i] = 0;
  }

  // 先清空CAN缓冲区
  clear_can_buffer();

  for (uint8_t motor_id = scan_start_id; motor_id <= scan_end_id; motor_id++) {
    Serial.printf("  检查电机ID: %d ... ", motor_id);

    bool found = false;
    for (int attempt = 0; attempt < 2; attempt++) {  // 减少尝试次数，因为ping_motor内部已经有了详细输出
      if (ping_motor(motor_id)) {
        discovered_motors[motor_count++] = motor_id;
        Serial.println("✓ 发现");
        found = true;
        break;
      }
      delay(150); // 增加延时
    }

    if (!found) {
      Serial.println("✗ 无响应");
    }

    delay(100); // 延时避免总线冲突
  }

  Serial.printf("\n扫描完成！发现 %d 个电机:\r\n", motor_count);
  if (motor_count > 0) {
    Serial.print("发现的电机ID: ");
    for (int i = 0; i < motor_count; i++) {
      Serial.print(discovered_motors[i]);
      if (i < motor_count - 1) Serial.print(", ");
    }
    Serial.println();
    Serial.println("\n💡 提示: 如果发现电机数量比预期少，可能的原因:");
    Serial.println("   1. 总线上有多个电机，ID冲突");
    Serial.println("   2. 电机电源未开启");
    Serial.println("   3. CAN连接问题");
  } else {
    Serial.println("未发现任何电机，请检查:");
    Serial.println("   1. CAN总线连接");
    Serial.println("   2. 电机电源(12-24V)");
    Serial.println("   3. 电机是否响应其他命令");
    Serial.println("   4. CAN总线波特率是否为1Mbps");
  }
}

// 清空CAN接收缓冲区
void clear_can_buffer() {
  CanFrame rxFrame;
  while (ESP32Can.readFrame(rxFrame, 10)) {
    // 丢弃所有现有消息
  }
}

bool ping_motor(uint8_t motor_id) {
  // 创建临时电机对象用于测试通信
  MI_Motor_ temp_motor;
  temp_motor.Motor_Con_Init(motor_id);

  // 使用发送反馈命令测试通信（与simple_joint_control相同的方式）
  can_frame_t frame;
  frame.mode = 2; // MOTOR_FEEDBACK
  frame.id = motor_id;
  frame.data = MASTER_ID;
  for (int i = 0; i < 8; i++) {
    frame.tx_data[i] = 0;
  }

  // 发送命令
  CAN_Send_Frame(&frame);
  delay(200); // 增加延时，给电机更多响应时间

  // 等待响应
  CanFrame rxFrame;
  if (ESP32Can.readFrame(rxFrame, 500)) { // 增加超时时间
    // 检查是否是来自目标电机的响应
    uint8_t rx_motor_id = (rxFrame.identifier >> 8) & 0xFF;
    uint8_t rx_master_id = rxFrame.identifier & 0xFF;

    // 只在调试模式下打印详细信息
    #ifdef DEBUG_CAN
    Serial.printf("收到响应: 0x%08X, 电机ID: %d, 主机ID: %d\r\n",
                     rxFrame.identifier, rx_motor_id, rx_master_id);
    #endif

    if (rx_motor_id == motor_id && rx_master_id == MASTER_ID) {
      return true;
    }
  }

  return false;
}

void process_set_command(String command) {
  // 解析 "set <old_id> <new_id>" 命令
  int first_space = command.indexOf(' ');
  int second_space = command.indexOf(' ', first_space + 1);

  if (second_space == -1) {
    Serial.println("❌ 格式错误，应为: set <old_id> <new_id>");
    return;
  }

  uint8_t old_id = command.substring(first_space + 1, second_space).toInt();
  uint8_t new_id = command.substring(second_space + 1).toInt();

  if (old_id < 1 || old_id > 127 || new_id < 1 || new_id > 127) {
    Serial.println("❌ ID范围错误，应为1-127");
    return;
  }

  if (old_id == new_id) {
    Serial.println("⚠️ 新ID与旧ID相同，无需修改");
    return;
  }

  set_motor_id(old_id, new_id);
}

void set_motor_id(uint8_t old_id, uint8_t new_id) {
  Serial.printf("修改电机ID: %d -> %d\r\n", old_id, new_id);
  Serial.println("⚠️  确保总线上只连接这一个电机！");

  // 创建电机对象
  MI_Motor_ motor;
  motor.Motor_Con_Init(old_id);

  // 发送设置电机ID命令 (模式7)
  can_frame_t frame;
  frame.mode = 7; // SET_MOTOR_CAN_ID
  frame.id = old_id;
  frame.data = MASTER_ID;

  // 数据区设置新电机ID
  frame.tx_data[0] = new_id;
  for (int i = 1; i < 8; i++) {
    frame.tx_data[i] = 0;
  }

  // 发送命令
  CAN_Send_Frame(&frame);
  delay(200);

  // 验证修改是否成功
  Serial.println("验证修改结果...");
  delay(300);

  if (ping_motor(new_id)) {
    Serial.printf("✓ 成功！电机ID已修改为: %d\r\n", new_id);
  } else {
    Serial.println("❌ 修改失败，请检查连接或重试");
  }
}

void process_batch_command(String command) {
  // 解析 "batch <start_id>" 命令
  int space_pos = command.indexOf(' ');
  if (space_pos == -1) {
    Serial.println("❌ 格式错误，应为: batch <start_id>");
    return;
  }

  uint8_t start_id = command.substring(space_pos + 1).toInt();
  if (start_id < 1 || start_id > 127) {
    Serial.println("❌ 起始ID范围错误，应为1-127");
    return;
  }

  Serial.printf("批量设置模式，从ID=%d开始\r\n", start_id);
  Serial.println("请依次连接电机，每个电机设置完成后按任意键继续...");
  Serial.println("输入 'stop' 结束批量设置");

  uint8_t current_id = start_id;
  while (current_id <= 127) {
    Serial.printf("\n=== 设置电机 #%d ===\r\n", current_id - start_id + 1);
    Serial.println("请将电机连接到总线，然后输入任意键继续设置...");
    Serial.println("或输入 'stop' 结束批量设置");

    // 等待用户确认
    while (!Serial.available()) {
      delay(100);
    }

    String user_input = Serial.readStringUntil('\n');
    user_input.trim();

    if (user_input.equalsIgnoreCase("stop")) {
      Serial.println("批量设置已取消");
      return;
    }

    // 扫描当前连接的电机
    scan_motors();

    if (motor_count == 0) {
      Serial.println("未发现电机，请检查连接");
      continue;
    }

    if (motor_count > 1) {
      Serial.println("⚠️ 发现多个电机，请只连接一个电机进行设置");
      continue;
    }

    // 获取发现的电机ID
    uint8_t found_id = discovered_motors[0];
    Serial.printf("发现电机ID: %d，将修改为: %d\r\n", found_id, current_id);

    // 设置新ID
    set_motor_id(found_id, current_id);

    current_id++;

    if (current_id > 127) {
      Serial.println("已达到最大ID数量(127)，批量设置完成");
      break;
    }
  }
}

void process_scan_range_command(String command) {
  // 解析 "scan <start> <end>" 命令
  int first_space = command.indexOf(' ');
  int second_space = command.indexOf(' ', first_space + 1);

  if (second_space == -1) {
    Serial.println("❌ 格式错误，应为: scan <start> <end>");
    return;
  }

  scan_start_id = command.substring(first_space + 1, second_space).toInt();
  scan_end_id = command.substring(second_space + 1).toInt();

  if (scan_start_id < 1 || scan_start_id > 127 ||
      scan_end_id < 1 || scan_end_id > 127 ||
      scan_start_id > scan_end_id) {
    Serial.println("❌ ID范围错误，应为1-127且start <= end");
    return;
  }

  scan_motors();
}

void process_reset_command(String command) {
  // 解析 "reset <id>" 命令
  int space_pos = command.indexOf(' ');
  if (space_pos == -1) {
    Serial.println("❌ 格式错误，应为: reset <id>");
    return;
  }

  uint8_t motor_id = command.substring(space_pos + 1).toInt();

  if (motor_id < 1 || motor_id > 127) {
    Serial.println("❌ ID范围错误，应为1-127");
    return;
  }

  Serial.printf("重置电机ID: %d\r\n", motor_id);

  // 创建电机对象
  MI_Motor_ motor;
  motor.Motor_Con_Init(motor_id);

  // 发送重置命令
  motor.Motor_Reset();
  delay(500);

  Serial.println("✓ 电机重置完成");
}

void verify_motors() {
  Serial.println("验证当前电机ID设置...");

  // 使用快速扫描模式
  scan_start_id = 1;
  scan_end_id = 20; // 通常前20个ID足够

  motor_count = 0;
  for (int i = 0; i < MAX_MOTORS; i++) {
    discovered_motors[i] = 0;
  }

  MI_Motor_ temp_motor;

  for (uint8_t motor_id = scan_start_id; motor_id <= scan_end_id; motor_id++) {
    Serial.printf("  检查ID %d ... ", motor_id);

    temp_motor.Motor_Con_Init(motor_id);

    if (ping_motor(motor_id)) {
      discovered_motors[motor_count++] = motor_id;
      Serial.println("✓ 响应");
    } else {
      Serial.println("✗ 无响应");
    }

    delay(50);
  }

  Serial.printf("\n验证结果：发现 %d 个电机\r\n", motor_count);

  if (motor_count > 0) {
    Serial.print("当前电机ID: ");
    for (int i = 0; i < motor_count; i++) {
      Serial.printf("%d", discovered_motors[i]);
      if (i < motor_count - 1) Serial.print(", ");
    }
    Serial.println();

    // 建议下一步操作
    if (motor_count < 3) {
      Serial.println("💡 建议：电机数量较少，可以继续添加更多电机");
    } else {
      Serial.println("✅ 电机数量充足，可以开始多电机控制");
    }
  } else {
    Serial.println("❌ 未发现任何电机，请检查：");
    Serial.println("   1. CAN总线连接");
    Serial.println("   2. 电机电源");
    Serial.println("   3. 电机ID是否正确");
  }
}

// 测试CAN通信功能
void test_can_communication() {
  Serial.println("=== 测试CAN通信 ===");
  Serial.println("测试与电机的通信连接...");

  // 先快速扫描1-20，看有没有电机
  bool found_any = false;
  uint8_t found_id = 0;

  Serial.println("正在扫描电机ID 1-20...");
  for (uint8_t motor_id = 1; motor_id <= 20; motor_id++) {
    if (ping_motor(motor_id)) {
      found_any = true;
      found_id = motor_id;
      break;  // 找到一个就够了
    }
    delay(50);  // 短延时
  }

  if (found_any) {
    Serial.printf("✅ CAN通信正常！发现电机ID=%d\r\n", found_id);
    Serial.println("可以继续使用扫描功能查找所有电机");
    Serial.println("提示：如果这个ID不是您期望的，可以使用scan命令查看所有电机");
  } else {
    Serial.println("❌ CAN通信失败！未发现任何电机");
    Serial.println("请检查：");
    Serial.println("1. CAN总线连接是否正确");
    Serial.println("2. 电机电源是否开启(12-24V)");
    Serial.println("3. 波特率是否为1Mbps");
    Serial.println("4. 终端电阻是否配置");
    Serial.println("5. 总线上是否连接了电机");
    Serial.println();
    Serial.println("💡 如果您刚刚修改了电机ID，请尝试：");
    Serial.println("  scan 1 127   # 扫描完整范围");
    Serial.println("  scan 10 20   # 扫描您设置的新ID范围");
  }

  Serial.println("==================");
}

// 底层CAN发送函数（复制自TWAI_CAN_MI_Motor.cpp）
static void CAN_Send_Frame(can_frame_t* frame) {
    CanFrame obdFrame = { 0 };
    uint32_t id_val, data_val, mode_val;
    uint32_t combined_val;

    obdFrame.extd = 1;              //0-标准帧; 1-扩展帧
    obdFrame.rtr = 0;               //0-数据帧; 1-远程帧
    obdFrame.ss = 0;                //0-错误重发; 1-单次发送
    obdFrame.self = 0;              //0-不接收自己发送的消息
    obdFrame.dlc_non_comp = 0;      //0-数据长度不大于8

    //拼接ID
    id_val = frame->id;
    data_val = frame->data;
    mode_val = frame->mode;
    combined_val |= (mode_val << 24);
    combined_val |= (data_val << 8);
    combined_val |= id_val;

    obdFrame.identifier = combined_val;
    obdFrame.data_length_code = 8;

    for (int i = 0; i < 8; i++) {
        obdFrame.data[i] = frame->tx_data[i];
    }
    ESP32Can.writeFrame(obdFrame);
}