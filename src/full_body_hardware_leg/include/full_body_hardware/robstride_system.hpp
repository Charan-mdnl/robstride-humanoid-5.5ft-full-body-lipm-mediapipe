#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include <algorithm>  // std::min, std::max
#include <cmath>      // std::sin, std::sqrt, std::abs
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <chrono>
#include "full_body_hardware/can_interfaces.h"
#include "full_body_hardware/protocol_completed.h"

namespace full_body_hardware
{

// ─────────────────────────────────────────────────────────────────────────────
// Encode / Decode helpers (MIT mode — Big-Endian 16-bit fixed-point)
// ─────────────────────────────────────────────────────────────────────────────

// TODO (Step 5a): Implement encode_signed.
//   Converts a physical value in [-max_val, +max_val] to a uint16_t [0x0000, 0xFFFF].
//   Formula: raw = uint16_t( ((val / max_val) + 1.0) * 0x7FFF )
//   Clamp val to [-max_val, max_val] before the formula to prevent overflow.
inline uint16_t encode_signed(double val, double max_val)
{
    // TODO: implement
    uint16_t raw = 0;
    double clamped = std::max(0.0, std::min(65535.0, (val / max_val + 1.0) * 0x7FFF));
    raw = static_cast<uint16_t>(clamped);
    return raw;
}

// TODO (Step 5b): Implement encode_unsigned.
//   Converts a physical value in [0, max_val] to a uint16_t [0x0000, 0xFFFF].
//   Formula: raw = uint16_t( (val / max_val) * 0xFFFF )
//   Clamp val to [0, max_val].
inline uint16_t encode_unsigned(double val, double max_val)
{
    // TODO: implement
    uint16_t raw = 0;
    raw = static_cast<uint16_t>(std::max(0.0,std::min(65535.0,(val/max_val)*0xFFFF)));

    return raw;
}

// TODO (Step 6a): Implement decode_signed.
//   Inverse of encode_signed.
//   Formula: val = (raw / double(0x7FFF) - 1.0) * max_val
inline double decode_signed(uint16_t raw, double max_val)
{
    // TODO: implement
     //raw = std::max(0,std::min(65535, raw));
     double val = (raw / double(0x7FFF) - 1.0) * max_val;
    // (void)raw;

     return val;
}

// TODO (Step 6b): Implement decode_unsigned.
//   Inverse of encode_unsigned.
//   Formula: val = (raw / double(0xFFFF)) * max_val
inline double decode_unsigned(uint16_t raw, double max_val)
{
    // TODO: implement
    double val = (raw/double(0xFFFF)) * max_val;
    //(void)raw;
    return val;
}

// ─────────────────────────────────────────────────────────────────────────────
// RobStrideSystem — ros2_control SystemInterface for RobStride CAN actuators
// ─────────────────────────────────────────────────────────────────────────────
class RobStrideSystem : public hardware_interface::SystemInterface
{
public:
    RobStrideSystem();
    ~RobStrideSystem() override;      // <-- add this

    // ── Lifecycle callbacks ──────────────────────────────────────────────────
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    // ── Interface export ─────────────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // ── Control loop ─────────────────────────────────────────────────────────
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // TODO (Step 2a): Uncomment once can_interfaces.h is included above.
    //   CanInterface can_;
//    CanInterface can_;
struct MotorBus{
std::string name;
CanInterface can;
std::vector<size_t> idx;
std::thread worker;
bool first_write{true};
std::vector<double> local_pos;
};
std::vector<std::unique_ptr<MotorBus>> buses_;
std::unordered_map<uint8_t,size_t> id_to_idx;
std::atomic<bool> run_{false};
int update_rate_hz_{200};
std::mutex data_mtx_;
std::vector<double> cmd_shared_,st_pos_shared_,st_vel_shared_,st_torque_shared_;
void bus_loop(MotorBus & bus);
void activate_bus(MotorBus & bus);
    // ── CAN bus configuration ────────────────────────────────────────────────
    // Populated from hardware <param name="can_interface">can0</param> in the xacro.
    std::string can_interface_name_;

    // ── Per-joint CAN IDs ───────────────────────────────────────────────────
    // TODO (Step 2b): Populated in on_init from <param name="motor_id"> in each joint.
    //   NOTE: The xacro currently has NO motor_id params — you will need to add them.
    //   Example addition to the ctrl_joint macro in full_body.ros2_control.xacro:
    //     <param name="motor_id">11</param>
    //     <param name="motor_model">rs03</param>
    std::vector<uint8_t> motor_ids_;
    std::vector<std::string> motor_models_;  // e.g. "rs03", "rs01", "rs02" — used to look up scaling factors in ModelScale namespaces.

    // ── Per-joint model scaling (from ModelScale::RS03:: etc.) ──────────────
    // TODO (Step 2c): Populated in on_init after reading motor_model param.
    //   Map model string → ModelScale namespace values.
    std::vector<double> motor_scale_pos_;     // POSITION scale (rad), e.g. 4π
    std::vector<double> motor_scale_vel_;     // VELOCITY scale (rad/s)
    std::vector<double> motor_scale_torque_;  // TORQUE   scale (Nm)
    std::vector<double> motor_scale_kp_;      // KP       scale (Nm/rad)
    std::vector<double> motor_scale_kd_;      // KD       scale (Nm·s/rad)

    // ── State buffers (written by read(), read by controllers) ───────────────
    std::vector<double> pos_state_;     // position  (rad)
    std::vector<double> vel_state_;     // velocity  (rad/s)
    std::vector<double> torque_state_;  // torque    (Nm)

    // ── Command buffers (written by controllers, read by write()) ────────────
    std::vector<double> pos_cmd_;       // position command (rad)
    std::vector<bool>   invert_;        // true → negate command and state

    // ── Per-joint feedforward physical params ────────────────────────────────
    // kp and kd are computed each write() cycle from these:
    //   tau_ff  = mass * 9.81 * length * sin(theta + offset)
    //   kp      = max(kp_min_, |tau_ff| / e_nominal)
    //   kd      = min(kd_max,  2 * sqrt(kp * inertia))
    std::vector<double> mass_;       // link mass (kg)
    std::vector<double> length_;     // moment arm (m)
    std::vector<double> offset_;     // theta offset (rad)
    std::vector<double> inertia_;    // rotational inertia (kg·m²)
    std::vector<double> e_nominal_;  // nominal error (rad) for kp = tau/e
    std::vector<double> kd_max_;     // kd cap (Nm·s/rad), default 100
    static constexpr double kp_min_ = 10.0;  // floor to avoid singularity at theta=0

    // Set to true initially; cleared after the first write() call.
    // Used to re-enable motors that may have hit their CAN watchdog timeout
    // during the sequential hardware-block activation gap.
    bool first_write_{true};
};

}  // namespace full_body_hardware
