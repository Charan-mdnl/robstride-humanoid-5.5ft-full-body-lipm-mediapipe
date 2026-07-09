/*
 * RobStride Motor Control Protocol Definitions
 * Communication protocol constants and data structures
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <cstring>
#include <cmath>

// --- Communication Types ---
namespace CommType {
    constexpr uint32_t GET_DEVICE_ID       = 0;
    constexpr uint32_t OPERATION_CONTROL   = 1;
    constexpr uint32_t OPERATION_STATUS    = 2;
    constexpr uint32_t ENABLE              = 3;
    constexpr uint32_t DISABLE             = 4;
    constexpr uint32_t SET_ZERO_POSITION   = 6;
    constexpr uint32_t SET_DEVICE_ID       = 7;
    constexpr uint32_t READ_PARAMETER      = 17;
    constexpr uint32_t WRITE_PARAMETER     = 18;
    constexpr uint32_t FAULT_REPORT        = 21;
    constexpr uint32_t SAVE_PARAMETERS     = 22;
    constexpr uint32_t SET_BAUDRATE        = 23;
    constexpr uint32_t ACTIVE_REPORT       = 24;
    constexpr uint32_t SET_PROTOCOL        = 25;
}

// --- Parameter IDs ---
namespace ParamID {
    constexpr uint16_t MECHANICAL_OFFSET       = 0x2005;
    constexpr uint16_t MEASURED_POSITION       = 0x3016;
    constexpr uint16_t MEASURED_VELOCITY       = 0x3017;
    constexpr uint16_t MEASURED_TORQUE         = 0x302C;
    constexpr uint16_t MODE                    = 0x7005;
    constexpr uint16_t IQ_TARGET               = 0x7006;
    constexpr uint16_t VELOCITY_TARGET         = 0x700A;
    constexpr uint16_t TORQUE_LIMIT            = 0x700B;
    constexpr uint16_t CURRENT_KP              = 0x7010;
    constexpr uint16_t CURRENT_KI              = 0x7011;
    constexpr uint16_t CURRENT_FILTER_GAIN     = 0x7014;
    constexpr uint16_t POSITION_TARGET         = 0x7016;
    constexpr uint16_t VELOCITY_LIMIT          = 0x7017;
    constexpr uint16_t CURRENT_LIMIT           = 0x7018;
    constexpr uint16_t MECHANICAL_POSITION     = 0x7019;
    constexpr uint16_t IQ_FILTERED             = 0x701A;
    constexpr uint16_t MECHANICAL_VELOCITY     = 0x701B;
    constexpr uint16_t VBUS                    = 0x701C;
    constexpr uint16_t POSITION_KP             = 0x701E;
    constexpr uint16_t VELOCITY_KP             = 0x701F;
    constexpr uint16_t VELOCITY_KI             = 0x7020;
    constexpr uint16_t VELOCITY_FILTER_GAIN    = 0x7021;
    constexpr uint16_t VEL_ACCELERATION_TARGET = 0x7022;
    constexpr uint16_t PP_VELOCITY_MAX         = 0x7024;
    constexpr uint16_t PP_ACCELERATION_TARGET  = 0x7025;
    constexpr uint16_t EPSCAN_TIME             = 0x7026;
    constexpr uint16_t CAN_TIMEOUT             = 0x7028;
    constexpr uint16_t ZERO_STATE              = 0x7029;
}

// --- Motor Model Scaling Tables (RS-03) ---
namespace ModelScale {
    // For MIT mode scaling
    constexpr double POSITION = 4.0 * M_PI;    // ±4π radians
    constexpr double VELOCITY = 50.0;          // ±50 rad/s
    constexpr double TORQUE = 60.0;            // ±60 Nm
    constexpr double KP = 5000.0;              // 0-5000 Nm/rad
    constexpr double KD = 100.0;               // 0-100 Nm/rad/s
}

// --- Control Modes ---
namespace ControlMode {
    constexpr int8_t MIT_MODE        = 0;      // MIT control mode
    constexpr int8_t POSITION_MODE   = 1;      // Position control mode
    constexpr int8_t SPEED_MODE      = 2;      // Speed control mode
    constexpr int8_t TORQUE_MODE     = 3;      // Torque control mode
}

// --- Utility Functions ---
inline void pack_float_le(uint8_t* buf, float val) {
    memcpy(buf, &val, sizeof(float));
}

inline void pack_u16_le(uint8_t* buf, uint16_t val) {
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
}

inline void pack_u16_be(uint8_t* buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

#endif // PROTOCOL_H