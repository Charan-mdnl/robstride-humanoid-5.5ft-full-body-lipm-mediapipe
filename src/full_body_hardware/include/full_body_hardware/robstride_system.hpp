#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include <algorithm>  // std::min, std::max
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <unordered_map>

#include "full_body_hardware/can_interfaces.h"
#include "full_body_hardware/protocol_completed.h"

namespace full_body_hardware
{

// ─────────────────────────────────────────────────────────────────────────────
// Encode / Decode helpers (MIT mode — Big-Endian 16-bit fixed-point)
// ─────────────────────────────────────────────────────────────────────────────

// Converts a physical value in [-max_val, +max_val] to a uint16_t [0x0000, 0xFFFF].
inline uint16_t encode_signed(double val, double max_val)
{
    uint16_t raw = 0;
    double clamped = std::max(0.0, std::min(65535.0, (val / max_val + 1.0) * 0x7FFF));
    raw = static_cast<uint16_t>(clamped);
    return raw;
}

// Converts a physical value in [0, max_val] to a uint16_t [0x0000, 0xFFFF].
inline uint16_t encode_unsigned(double val, double max_val)
{
    uint16_t raw = 0;
    raw = static_cast<uint16_t>(std::max(0.0,std::min(65535.0,(val/max_val)*0xFFFF)));
    return raw;
}

// Inverse of encode_signed.
inline double decode_signed(uint16_t raw, double max_val)
{
     double val = (raw / double(0x7FFF) - 1.0) * max_val;
     return val;
}

// Inverse of encode_unsigned.
inline double decode_unsigned(uint16_t raw, double max_val)
{
    double val = (raw/double(0xFFFF)) * max_val;
    return val;
}

// ─────────────────────────────────────────────────────────────────────────────
// RobStrideSystem — ros2_control SystemInterface for RobStride CAN actuators
//
//   Motors are grouped by the CAN bus they live on (per-joint
//   <param name="can_interface">).  Each bus gets its own socket AND its own
//   worker thread, so both arms transact in parallel.  The ros2_control
//   read()/write() calls only exchange data with the workers through
//   mutex-guarded shared buffers — no CAN I/O happens on the control thread.
// ─────────────────────────────────────────────────────────────────────────────
class RobStrideSystem : public hardware_interface::SystemInterface
{
public:
    RobStrideSystem();
    ~RobStrideSystem() override;

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
    // ── One CAN bus = one socket + one worker thread ─────────────────────────
    struct MotorBus
    {
        std::string          name;      // e.g. "can2"
        CanInterface         can;       // owns the socket fd (non-copyable)
        std::vector<size_t>  idx;       // indices into the global per-joint vectors
        std::thread          worker;    // background transaction loop
    };

    // The per-bus worker loop: snapshot commands → write MIT frames → read &
    // dispatch STATUS frames → publish state. Runs until run_ goes false.
    void bus_loop(MotorBus & bus);

    // Bring a single bus online (zero / save / enable / MIT-mode / seed state).
    // Runs on the activating thread BEFORE the worker starts, so no locking.
    bool activate_bus(MotorBus & bus);

    // Encode the MIT operation-control frame for joint i into (ext_id, data[8]).
    void build_mit_frame(size_t i, double pos_cmd, uint32_t & ext_id, uint8_t data[8]) const;

    // ── CAN buses (one per unique can_interface across all joints) ───────────
    std::vector<std::unique_ptr<MotorBus>> buses_;
    std::unordered_map<uint8_t, size_t>    id_to_idx_;   // motor_id → joint index

    std::atomic<bool> run_{false};       // worker loops spin while true
    int               update_rate_hz_ = 200;

    // Default bus if a joint omits <param name="can_interface">.
    std::string can_interface_name_;

    // ── Per-joint CAN IDs / model ────────────────────────────────────────────
    std::vector<uint8_t>     motor_ids_;
    std::vector<std::string> motor_models_;

    // ── Per-joint model scaling (from ModelScale::RS03:: etc.) ──────────────
    std::vector<double> motor_scale_pos_;
    std::vector<double> motor_scale_vel_;
    std::vector<double> motor_scale_torque_;
    std::vector<double> motor_scale_kp_;
    std::vector<double> motor_scale_kd_;

    // ── State buffers exposed to controllers (owned by control thread) ───────
    std::vector<double> pos_state_;
    std::vector<double> vel_state_;
    std::vector<double> torque_state_;

    // ── Command buffer exposed to controllers (owned by control thread) ──────
    std::vector<double> pos_cmd_;
    std::vector<bool>   invert_;

    // ── Per-joint MIT gains ──────────────────────────────────────────────────
    std::vector<double> kp_;
    std::vector<double> kd_;

    // ── Shared hand-off buffers between control thread and workers ───────────
    //   write() copies pos_cmd_ → cmd_shared_ ; workers read cmd_shared_.
    //   workers write st_*_shared_ ; read() copies st_*_shared_ → *_state_.
    std::mutex          data_mtx_;
    std::vector<double> cmd_shared_;
    std::vector<double> st_pos_shared_;
    std::vector<double> st_vel_shared_;
    std::vector<double> st_torque_shared_;
};

}  // namespace full_body_hardware
