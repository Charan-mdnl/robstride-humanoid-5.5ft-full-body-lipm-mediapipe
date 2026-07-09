/*
 * hello_motor.cpp — Build this file yourself, step by step.
 *
 * GOAL: Send one position command to one motor and print what it reports back.
 *
 * Build when ready:
 *   g++ -std=c++17 -I../include -o hello_motor hello_motor.cpp
 *
 * Run:
 *   sudo ./hello_motor <motor_id> <degrees>
 *   e.g.  sudo ./hello_motor 11 45
 *
 * ============================================================
 *  WHAT YOU NEED TO KNOW BEFORE WRITING ANY CODE
 * ============================================================
 *
 *  1. CAN ID LAYOUT (29-bit extended)
 *     Each message has a 29-bit ID that carries meaning:
 *
 *       bits [28:24]  comm_type   — what command? (5 bits)
 *       bits [23:16]  extra       — depends on command
 *       bits [15:8]   sender_id   — who is sending  (PC = 0xFF)
 *       bits [7:0]    motor_id    — who should act
 *
 *     Example — ENABLE (type=3) to motor 11:
 *       ext_id = (3 << 24) | (0 << 16) | (0xFF << 8) | 11
 *
 *  2. COMM TYPES (from protocol_completed.h → CommType namespace)
 *     ENABLE           = 3    no data, wakes motor
 *     DISABLE          = 4    no data, idles motor
 *     WRITE_PARAMETER  = 18   8 bytes: [param_id LE 2B][padding 2B][value 4B]
 *     OPERATION_CONTROL= 1    8 bytes: position/vel/kp/kd packed as uint16 BE
 *                             torque_ff goes in CAN ID bits [23:8]
 *     OPERATION_STATUS = 2    reply FROM motor: pos/vel/torque/temp/fault
 *
 *  3. PARAMETER IDS (from protocol_completed.h → ParamID namespace)
 *     ParamID::MODE     = 0x7005   int8:  0=MIT, 1=Position, 2=Speed, 3=Torque
 *     ParamID::VELOCITY_LIMIT = 0x7017  float: rad/s
 *     ParamID::TORQUE_LIMIT   = 0x700B  float: Nm
 *
 *  4. SCALE TABLES (from protocol_completed.h → ModelScale::RS03)
 *     POSITION = 4π  rad      signed ±
 *     VELOCITY = 50  rad/s    signed ±
 *     TORQUE   = 60  Nm       signed ±
 *     KP       = 5000 Nm/rad  unsigned 0–max
 *     KD       = 100  Nm·s/rad unsigned 0–max
 *
 *  5. ENCODING FORMULAS
 *     Signed value  (pos, vel, torque):
 *       raw_u16 = (value / max + 1.0) * 32767      range → [0, 65534]
 *       0 maps to 32767 (midpoint)
 *
 *     Unsigned value (kp, kd):
 *       raw_u16 = (value / max) * 65535             range → [0, 65535]
 *
 *     DECODING (what motor replies):
 *       value = (raw_u16 / 32767.0 - 1.0) * max
 *
 *  6. BYTE ORDER
 *     Control frame data bytes   → BIG-endian    (use pack_u16_be)
 *     WRITE_PARAMETER param_id   → LITTLE-endian (use pack_u16_le)
 *     WRITE_PARAMETER float val  → LITTLE-endian (use pack_float_le / memcpy)
 *
 *  7. USEFUL API FROM can_interfaces.h
 *     CanInterface can;
 *     can.init("can0");                            // open the socket
 *     can.write_frame(ext_id, data, dlc);          // send a frame
 *     can.read_frame(&frame, timeout_ms);          // receive a frame (returns bool)
 *     can.is_ready();                              // check if socket open
 *
 *     The received frame is a Linux struct can_frame:
 *       frame.can_id   — full 29-bit ID with CAN_EFF_FLAG set
 *       frame.can_dlc  — number of data bytes
 *       frame.data[]   — up to 8 payload bytes
 *
 *     To extract fields from a received can_id:
 *       uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
 *       int      src_id    = (frame.can_id >>  8) & 0xFF;
 *
 * ============================================================
 *  YOUR TASKS — fill in each section below
 * ============================================================
 */

#include "../include/motor_control.hpp"

// torque — motor can be rotated freely by hand.

int main(int argc, char* argv[])
{
    if(argc < 5)
    {
        std::cerr << "Usage: sudo ./hello_motor <motor_id> <target_deg> <motor_type> <zero>" << std::endl;
        return -1;
    }
    int motor_id = std::atoi(argv[1]);
    double target_deg = std::atof(argv[2]);
    char z = char(argv[4][0]);
    std::string motor_type = std::string(argv[3]);
    double target_rad = target_deg*M_PI/180.0;
    CanInterface can;
    if(!can.init("can0") || !can.is_ready())
    {
        std::cerr << "Failed to initialize CAN interface" << std::endl;
        return -1;
    }
    switch (z)
    {
    
    case 'z':
    {
        set_zero_position(can, motor_id);
        // return if nothing
        struct can_frame tmp;
        while (can.read_frame(&tmp, 5))
        {
        }

        return 0;
        /* code */
        break;
    }
    case 'r':
    {
        send_enable(can, motor_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        send_mode(can, motor_id, ControlMode::MIT_MODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        read_encoder_continuous(can, motor_id, 500, 4.0 * M_PI);
        uint32_t dis_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | motor_id;
        can.write_frame(dis_id, nullptr, 0);
        return 0;
    }
    case 'w':
    {
        send_enable(can, motor_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        send_mode(can, motor_id, ControlMode::MIT_MODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < 500; i++)
        {
            send_position(can, motor_id, target_rad, 5.0, 1.7, motor_type);
            struct can_frame frame;
            auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
            while (std::chrono::steady_clock::now() < deadline)
            {
                int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                             deadline - std::chrono::steady_clock::now())
                             .count();
                if (ms <= 0 || !can.read_frame(&frame, ms))
                    break;
                uint32_t ct = (frame.can_id >> 24) & 0x1F;
                int src = (frame.can_id >> 8) & 0xFF;
                if (ct == CommType::OPERATION_STATUS && src == motor_id)
                {
                    uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
                    uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
                    double p_scale, t_scale;
                    if (motor_type == "rs03")
                    {
                        p_scale = ModelScale::RS03::POSITION;
                        t_scale = ModelScale::RS03::TORQUE;
                    }
                    else if (motor_type == "rs02")
                    {
                        p_scale = ModelScale::RS02::POSITION;
                        t_scale = ModelScale::RS02::TORQUE;
                    }
                    else if (motor_type == "rs04")
                    {
                        p_scale = ModelScale::RS04::POSITION;
                        t_scale = ModelScale::RS04::TORQUE;
                    }
                    else if (motor_type == "rs01")
                    {
                        p_scale = ModelScale::RS01::POSITION;
                        t_scale = ModelScale::RS01::TORQUE;
                    }
                    else
                    {
                        p_scale = ModelScale::RS00::POSITION;
                        t_scale = ModelScale::RS00::TORQUE;
                    }
                    double pos = (pos_raw / 32767.0 - 1.0) * p_scale;
                    double torque = (torque_raw / 32767.0 - 1.0) * t_scale;
                    std::cout << "raw=" << pos_raw
                              << "  pos=" << pos * 180.0 / M_PI << " deg"
                              << "  target=" << target_deg << " deg"
                              << "  torque=" << torque << " Nm\n";
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        uint32_t dis_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | motor_id;
        can.write_frame(dis_id, nullptr, 0);
        return 0;
    }
    default:
        break;
    }
    // (z == "zero")
    // {
    //     set_zero_position(can, motor_id);
    //    // return if nothing
    //     struct can_frame tmp;
    //     while (can.read_frame(&tmp, 5))
    //     {
    //     }

    //     return 0;
    // }
    // else
    // {
    //     cout << "no set zero" << endl;
    // }
    std::cout << "CAN interface initialized successfully" << std::endl;
    send_enable(can,motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // if (z == "zero")
    // {
    //     set_zero_position(can, motor_id);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     uint32_t ext_id = (CommType::SAVE_PARAMETERS) | (HOST_ID << 8) | (motor_id);
    //     uint8_t save_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    //     can.write_frame(ext_id, save_data, 8);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //     struct can_frame tmp;
    //     while (can.read_frame(&tmp, 5))
    //     {
    //     }
    //     uint32_t dis_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | motor_id;
    //     can.write_frame(dis_id, nullptr, 0);
    //     std::cout << "Zero set and saved. Power cycle the motor to confirm." << std::endl;
    //     return 0;
    // }
    // else
    // {
    //     cout << "no set zero" << endl;
    // }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    send_mode(can,motor_id,ControlMode::MIT_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double pos,vel,torque;
    if(read_status(can,motor_id,pos,vel,torque,motor_type))
    {
        std::cout << "Current position: " << pos << " rad (" << pos*180.0/M_PI << " deg)" << std::endl;
    }
    else
    {
        std::cerr << "Failed to read status, check if the motor type is correct" << std::endl;
    }
    for (size_t i = 0; i < 50; i++)
    {    // zero feedforward
        send_position(can,motor_id,target_rad,5.0,1.7,motor_type);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Drain stale frames accumulated during the send loop
    {
        struct can_frame tmp;
        while (can.read_frame(&tmp, 5)) {}
    }
    // Trigger one fresh status reply
    send_position(can, motor_id, target_rad, 5.0, 1.7, motor_type);

    if (read_status(can, motor_id, pos, vel, torque, motor_type))
    {
        std::cout << "Current position: " << pos << " rad (" << pos * 180.0 / M_PI << " deg)" << std::endl;
    }
    else
    {
        std::cerr << "Failed to read status, check if the motor type is correct" << std::endl;
    }

    uint32_t dis_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | motor_id;
    can.write_frame(dis_id, nullptr, 0);
    return 0;
}
