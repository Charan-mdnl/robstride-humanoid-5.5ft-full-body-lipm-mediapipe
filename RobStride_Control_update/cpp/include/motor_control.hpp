#pragma once

#include <iostream>
#include <cmath>
#include <cstring>
#include <thread>
#include <chrono>
#include <string>
#include <algorithm>
#include "can_interfaces.h"
#include "protocol_completed.h"

#define HOST_ID 0xFF
// ---------------------------------------------------------------------------
// Motor — per-motor state used in multi-motor control loops
// ---------------------------------------------------------------------------
#include <vector>
struct Motor
{
    int id;
    std::string type;
    std::string can_interface = "can0";  // which CAN bus this motor is on
    double target_rad = 0.0;
    double torque_ff = 0.0;  // feedforward torque in Nm
    double kp = 5.0, kd = 1.7;
    uint16_t pos_raw = 0;
    double pos = 0, vel = 0, torque = 0;
    double filtered_vel = 0.0;
    bool vel_filter_initialized = false;
};

inline constexpr double VELOCITY_FILTER_ALPHA = 0.2;

inline void read_status_scales(const std::string &type,
                               uint16_t pos_raw, uint16_t vel_raw, uint16_t torque_raw,
                               double &pos, double &vel, double &torque)
{
    double p_max, v_max, t_max;
    if (type == "rs03")
    {
        p_max = ModelScale::RS03::POSITION;
        v_max = ModelScale::RS03::VELOCITY;
        t_max = ModelScale::RS03::TORQUE;
    }
    else if (type == "rs02")
    {
        p_max = ModelScale::RS02::POSITION;
        v_max = ModelScale::RS02::VELOCITY;
        t_max = ModelScale::RS02::TORQUE;
    }
    else if (type == "rs04")
    {
        p_max = ModelScale::RS04::POSITION;
        v_max = ModelScale::RS04::VELOCITY;
        t_max = ModelScale::RS04::TORQUE;
    }
    else if (type == "rs01")
    {
        p_max = ModelScale::RS01::POSITION;
        v_max = ModelScale::RS01::VELOCITY;
        t_max = ModelScale::RS01::TORQUE;
    }
    else
    {
        p_max = ModelScale::RS00::POSITION;
        v_max = ModelScale::RS00::VELOCITY;
        t_max = ModelScale::RS00::TORQUE;
    }
    pos = (pos_raw / 32767.0 - 1.0) * p_max;
    vel = (vel_raw / 32767.0 - 1.0) * v_max;
    torque = (torque_raw / 32767.0 - 1.0) * t_max;
}

inline void collect_status(CanInterface &can, std::vector<Motor> &motors,
                           int timeout_ms = 10)
{
    int received = 0;
    const int per_frame_timeout_ms = 2;  // very short per-frame timeout
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);
    
    while (received < (int)motors.size() &&
           std::chrono::steady_clock::now() < deadline)
    {
        struct can_frame frame;
        
        // Use short per-frame timeout to avoid blocking on any single read
        if (!can.read_frame(&frame, per_frame_timeout_ms))
            continue;  // Try again until deadline
            
        uint32_t ct = (frame.can_id >> 24) & 0x1F;
        int src = (frame.can_id >> 8) & 0xFF;
        if (ct != CommType::OPERATION_STATUS)
            continue;
            
        for (auto &m : motors)
        {
            if (src == m.id)
            {
                m.pos_raw = (uint16_t(frame.data[0]) << 8) | frame.data[1];
                uint16_t v_raw = (uint16_t(frame.data[2]) << 8) | frame.data[3];
                uint16_t t_raw = (uint16_t(frame.data[4]) << 8) | frame.data[5];
                read_status_scales(m.type, m.pos_raw, v_raw, t_raw, m.pos, m.vel, m.torque);
                if (!m.vel_filter_initialized)
                {
                    m.filtered_vel = m.vel;
                    m.vel_filter_initialized = true;
                }
                else
                {
                    m.filtered_vel = VELOCITY_FILTER_ALPHA * m.vel +
                                     (1.0 - VELOCITY_FILTER_ALPHA) * m.filtered_vel;
                }
                received++;
                break;
            }
        }
    }
}
// ---------------------------------------------------------------------------
// read_encoder_continuous
// Sends zero-torque MIT heartbeat frames and reads OPERATION_STATUS replies,
// tracking multi-turn angle with rollover detection on the 16-bit pos_raw.
// ---------------------------------------------------------------------------
inline void read_encoder_continuous(CanInterface &can, int motor_id, int iterations, double pos_scale)
{
    auto ping_and_get = [&]() -> int32_t
    {
        uint16_t torque_u16 = 0x7FFF;
        uint8_t data[8];
        pack_u16_be(&data[0], 0x7FFF); // pos (ignored when kp=0)
        pack_u16_be(&data[2], 0x7FFF); // vel (ignored when kd=0)
        pack_u16_be(&data[4], 0x0000); // kp = 0
        pack_u16_be(&data[6], 0x0000); // kd = 0
        uint32_t ctrl_id = (CommType::OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
        can.write_frame(ctrl_id, data, 8);

        struct can_frame frame;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        while (std::chrono::steady_clock::now() < deadline) {
            int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (ms <= 0 || !can.read_frame(&frame, ms)) break;
            uint32_t ct  = (frame.can_id >> 24) & 0x1F;
            int      src = (frame.can_id >> 8)  & 0xFF;
            if (ct == CommType::OPERATION_STATUS && src == motor_id) {
                uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
                return (int32_t)pos_raw;
            }
        }
        return -1;
    };

    int32_t prev = ping_and_get();
    if (prev < 0) { std::cerr << "No status frame received\n"; return; }

    int32_t total_counts = 0;

    for (int i = 0; i < iterations; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        int32_t curr = ping_and_get();
        if (curr < 0) continue;

        int32_t delta = curr - prev;
        if (delta >  32768) delta -= 65536;
        if (delta < -32768) delta += 65536;

        total_counts += delta;
        prev = curr;

        double rad = (total_counts / 32767.0) * pos_scale;
        double deg = rad * 180.0 / M_PI;
        std::cout << "raw=" << curr
                  << "  total=" << total_counts
                  << "  angle=" << deg << " deg  "
                  << rad << " rad\n";
    }
}

// ---------------------------------------------------------------------------
// read_encoder_raw
// Reads parameter 0x3004 (NOTE: returns 0 on RS02, effectively unused).
// ---------------------------------------------------------------------------
inline void read_encoder_raw(CanInterface &can, int motor_id)
{
    uint32_t ext_id = (CommType::READ_PARAMETER << 24) | (HOST_ID << 8) | (motor_id);
    uint8_t req_data[8] = {0};
    pack_u16_le(&req_data[0], 0x3004);
    can.write_frame(ext_id, req_data, 8);

    struct can_frame frame;
    if (can.read_frame(&frame, 200))
    {
        uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
        int src_id = (frame.can_id >> 8) & 0xFF;
        if (comm_type == CommType::READ_PARAMETER && src_id == motor_id)
        {
            uint16_t enc_raw;
            memcpy(&enc_raw, &frame.data[4], sizeof(uint16_t));
            double angle_deg = (enc_raw / 16384.0) * 360.0;
            std::cout << "Encoder raw: " << enc_raw << " / 16383  " << angle_deg << " deg\n";
        }
    }
}

// ---------------------------------------------------------------------------
// send_enable
// ---------------------------------------------------------------------------
inline void send_enable(CanInterface &can, int motor_id)
{
    uint32_t ext_id = (CommType::ENABLE << 24) | (HOST_ID << 8) | motor_id;
    can.write_frame(ext_id, nullptr, 0);
}

// ---------------------------------------------------------------------------
// send_mode
// ---------------------------------------------------------------------------
inline void send_mode(CanInterface &can, int motor_id, int8_t mode)
{
    uint32_t ext_id = (CommType::WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], ParamID::MODE);
    data[4] = (uint8_t)mode;
    can.write_frame(ext_id, data, 8);
}

// ---------------------------------------------------------------------------
// send_position
// Encodes pos/vel/kp/kd as uint16 BE and sends OPERATION_CONTROL.
// model: "rs00", "rs01", "rs02", "rs03", "rs04"
// torque_ff: feedforward torque in Nm (default 0.0 for zero torque)
// ---------------------------------------------------------------------------
inline void send_position(CanInterface &can, int motor_id, double pos_rad,
                          double kp, double kd, const std::string &scale,
                          double torque_ff = 0.0, double vel_rad_s = 0.0)
{
    double p_max, v_max, kp_max, kd_max, t_max;

    if (scale == "rs03") {
        p_max  = ModelScale::RS03::POSITION; v_max  = ModelScale::RS03::VELOCITY;
        kp_max = ModelScale::RS03::KP;       kd_max = ModelScale::RS03::KD;
        t_max  = ModelScale::RS03::TORQUE;
    } else if (scale == "rs02") {
        p_max  = ModelScale::RS02::POSITION; v_max  = ModelScale::RS02::VELOCITY;
        kp_max = ModelScale::RS02::KP;       kd_max = ModelScale::RS02::KD;
        t_max  = ModelScale::RS02::TORQUE;
    } else if (scale == "rs04") {
        p_max  = ModelScale::RS04::POSITION; v_max  = ModelScale::RS04::VELOCITY;
        kp_max = ModelScale::RS04::KP;       kd_max = ModelScale::RS04::KD;
        t_max  = ModelScale::RS04::TORQUE;
    } else if (scale == "rs01") {
        p_max  = ModelScale::RS01::POSITION; v_max  = ModelScale::RS01::VELOCITY;
        kp_max = ModelScale::RS01::KP;       kd_max = ModelScale::RS01::KD;
        t_max  = ModelScale::RS01::TORQUE;
    } else { // rs00
        p_max  = ModelScale::RS00::POSITION; v_max  = ModelScale::RS00::VELOCITY;
        kp_max = ModelScale::RS00::KP;       kd_max = ModelScale::RS00::KD;
        t_max  = ModelScale::RS00::TORQUE;
    }

    double torque_nm = std::max(-t_max, std::min(t_max, torque_ff));
    pos_rad   = std::max(-p_max, std::min(p_max, pos_rad));
    vel_rad_s = std::max(-v_max, std::min(v_max, vel_rad_s));
    kp        = std::max(0.0, std::min(kp_max, kp));
    kd        = std::max(0.0, std::min(kd_max, kd));

    uint16_t pos_u16    = (uint16_t)(((pos_rad / p_max) + 1.0) * 0x7FFF);
    uint16_t vel_u16    = (uint16_t)(((vel_rad_s / v_max) + 1.0) * 0x7FFF);
    uint16_t kp_u16     = (uint16_t)((kp  / kp_max) * 0xFFFF);
    uint16_t kd_u16     = (uint16_t)((kd  / kd_max) * 0xFFFF);
    uint16_t torque_u16 = (uint16_t)((torque_nm / t_max + 1.0) * 32767);

    uint8_t data[8];
    pack_u16_be(&data[0], pos_u16);
    pack_u16_be(&data[2], vel_u16);
    pack_u16_be(&data[4], kp_u16);
    pack_u16_be(&data[6], kd_u16);

    uint32_t ext_id = (CommType::OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
    can.write_frame(ext_id, data, 8);
}

// ---------------------------------------------------------------------------
// read_status
// Reads one OPERATION_STATUS frame and decodes pos/vel/torque.
// ---------------------------------------------------------------------------
inline bool read_status(CanInterface &can, int motor_id,
                        double &pos, double &vel, double &torque,
                        const std::string &scale)
{
    struct can_frame frame;
    if (!can.read_frame(&frame, 200)) return false;
    if (!(frame.can_id & CAN_EFF_FLAG)) return false;

    uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
    int src_id = (frame.can_id >> 8) & 0xFF;
    if (comm_type != CommType::OPERATION_STATUS || src_id != motor_id) return false;

    uint16_t pos_raw    = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
    uint16_t vel_raw    = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    uint16_t torque_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

    double p_max, v_max, t_max;
    if (scale == "rs03") {
        p_max = ModelScale::RS03::POSITION; v_max = ModelScale::RS03::VELOCITY; t_max = ModelScale::RS03::TORQUE;
    } else if (scale == "rs02") {
        p_max = ModelScale::RS02::POSITION; v_max = ModelScale::RS02::VELOCITY; t_max = ModelScale::RS02::TORQUE;
    } else if (scale == "rs04") {
        p_max = ModelScale::RS04::POSITION; v_max = ModelScale::RS04::VELOCITY; t_max = ModelScale::RS04::TORQUE;
    } else if (scale == "rs01") {
        p_max = ModelScale::RS01::POSITION; v_max = ModelScale::RS01::VELOCITY; t_max = ModelScale::RS01::TORQUE;
    } else {
        p_max = ModelScale::RS00::POSITION; v_max = ModelScale::RS00::VELOCITY; t_max = ModelScale::RS00::TORQUE;
    }

    pos    = (pos_raw    / 32767.0 - 1.0) * p_max;
    vel    = (vel_raw    / 32767.0 - 1.0) * v_max;
    torque = (torque_raw / 32767.0 - 1.0) * t_max;
    return true;
}

// ---------------------------------------------------------------------------
// set_zero_position
// Sends SET_ZERO_POSITION then SAVE_PARAMETERS.
// ---------------------------------------------------------------------------
inline void set_zero_position(CanInterface &can, uint8_t motor_id)
{
    uint32_t ext_id = (CommType::SET_ZERO_POSITION << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
    std::cout << "Sending SET_ZERO_POSITION: ext_id=0x" << std::hex << ext_id
              << " motor=" << std::dec << (int)motor_id << std::endl;
    can.write_frame(ext_id, data, 8);
    std::cout << "SET_ZERO_POSITION frame sent" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    uint32_t save_id = (CommType::SAVE_PARAMETERS << 24) | (HOST_ID << 8) | motor_id;
    uint8_t save_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    can.write_frame(save_id, save_data, 8);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// ---------------------------------------------------------------------------
// disable
// ---------------------------------------------------------------------------
inline void disable(CanInterface &can, int motor_id)
{
    uint32_t ext_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | motor_id;
    can.write_frame(ext_id, nullptr, 0);
}
