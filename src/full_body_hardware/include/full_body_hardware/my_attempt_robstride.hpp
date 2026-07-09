#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include <algorithm> // std::min, std::max
#include "full_body_hardware/protocol_completed.h"
#include "full_body_hardware/can_interfaces.h"

namespace full_body_hardware
{
    class RobstrideControl: public hardware_interface::SystemInterface
    {  public:
        RobstrideControl();
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interface();
    std::vector<hardware_interface::CommandInterface> export_command_interface();

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) ;
    hardware_interface::return_type read(const rclcpp::Time &timer, const rclcpp::Duration &period);

    private:
        CanInterface can_;
        std::string can_interface_name_;
        std::vector<uint8_t> motor_ids_;
        std::vector<std::string> motor_models_; // e.g. "rs03", "rs01", "rs02" — used to look up scaling factors in ModelScale namespaces.

        // ── Per-joint model scaling (from ModelScale::RS03:: etc.) ──────────────
        // TODO (Step 2c): Populated in on_init after reading motor_model param.
        //   Map model string → ModelScale namespace values.
        std::vector<double> motor_scale_pos_;    // POSITION scale (rad), e.g. 4π
        std::vector<double> motor_scale_vel_;    // VELOCITY scale (rad/s)
        std::vector<double> motor_scale_torque_; // TORQUE   scale (Nm)
        std::vector<double> motor_scale_kp_;     // KP       scale (Nm/rad)
        std::vector<double> motor_scale_kd_;     // KD       scale (Nm·s/rad)

        // ── State buffers (written by read(), read by controllers) ───────────────
        std::vector<double> pos_state_;    // position  (rad)
        std::vector<double> vel_state_;    // velocity  (rad/s)
        std::vector<double> torque_state_; // torque    (Nm)

        // ── Command buffers (written by controllers, read by write()) ────────────
        std::vector<double> pos_cmd_; // position command (rad)
        std::vector<bool> invert_;    // true → negate command and state

        // ── Per-joint MIT gains ──────────────────────────────────────────────────
        // TODO (Step 2d): Read from <param name="kp"> / <param name="kd"> in the xacro,
        //   or use sensible model-specific defaults if the param is absent.
        std::vector<double> kp_;
        std::vector<double> kd_;
    };
}