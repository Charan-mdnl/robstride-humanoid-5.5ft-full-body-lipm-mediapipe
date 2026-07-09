#include <full_body_hardware/my_attempt_robstride.hpp>
#include <set>
namespace full_body_hardware{

    RobstrideControl::RobstrideControl()
    {
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"), "Constructor called");
    }


    // ─────────────────────────────────────────────────────────────────────────────
    // on_init
    //   Called once when the plugin is loaded.
    //   Parse URDF <param> tags, validate configuration, size all vectors.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn RobstrideControl::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        size_t size_of_joints = info_.joints.size();
        can_interface_name_ = info_.hardware_parameters.count("can_interface")
                                  ? info_.hardware_parameters.at("can_interface")
                                  : "can0";
        for (size_t i = 0; i < size_of_joints;++i)
        {
            const auto &joint = info_.joints[i];
            try
            {
                motor_ids_.push_back(static_cast<uint8_t>(std::stoi(joint.parameters.at("motor_id"))));
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                return CallbackReturn::ERROR;
            }
            try{
                std::string motor_type = joint.parameters.at("motor_model");
                motor_models_.push_back(motor_type);
                if (motor_type == "rs03")
                {
                    motor_scale_pos_.push_back(ModelScale::RS03::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS03::VELOCITY);
                    motor_scale_kp_.push_back(ModelScale::RS03::KP);
                    motor_scale_kd_.push_back(ModelScale::RS03::KD);
                    motor_scale_torque_.push_back(ModelScale::RS03::TORQUE);
                }
                else if (motor_type == "rs04")
                {
                    motor_scale_pos_.push_back(ModelScale::RS04::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS04::VELOCITY);
                    motor_scale_kp_.push_back(ModelScale::RS04::KP);
                    motor_scale_kd_.push_back(ModelScale::RS04::KD);
                    motor_scale_torque_.push_back(ModelScale::RS04::TORQUE);
                }
                else if (motor_type == "rs02")
                {
                    motor_scale_pos_.push_back(ModelScale::RS02::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS02::VELOCITY);
                    motor_scale_kp_.push_back(ModelScale::RS02::KP);
                    motor_scale_kd_.push_back(ModelScale::RS02::KD);
                    motor_scale_torque_.push_back(ModelScale::RS02::TORQUE);
                }
                else if (motor_type == "rs00")
                {
                    motor_scale_pos_.push_back(ModelScale::RS00::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS00::VELOCITY);
                    motor_scale_kp_.push_back(ModelScale::RS00::KP);
                    motor_scale_kd_.push_back(ModelScale::RS00::KD);
                    motor_scale_torque_.push_back(ModelScale::RS00::TORQUE);
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Joint '%s' has unknown model '%s'", joint.name.c_str(), model.c_str());
                    return CallbackReturn::ERROR;
                }
                double kp_val = joint.parameters.count("kp") ? std::stod(joint.parameters.at("kp")) : 15.00;
                double kd_val = joint.parameters.count("kd") ? std::stod(joint.parameters.at("kd")) : 1.40;
                kp_.push_back(kp_val);
                kd_.push_back(kd_val);
                bool invert_val = joint.parameters.count("invert") && joint.parameters.at("invert") == "true";
                invert_.push_back(invert_val);
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
                return CallbackReturn::ERROR;
            }
        }
        pos_state_.resize(size_of_joints,0);
        vel_state_.resize(size_of_joints, 0);
        torque_state_.resize(size_of_joints, 0);
        pos_cmd_.resize(size_of_joints, 0.0);
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"),
                    "on_init OK — %zu joints, CAN interface: %s", n, can_interface_name_.c_str());
        return CallbackReturn::SUCCESS;
    }

    // on_configure
    //   Validate configuration before the hardware is activated.
    // ───────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn RobstrideControl::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        std::set<uint8_t> unique_ids(motor_ids_.begin(), motor_ids_.end());
        if (unique_ids.size() != motor_ids_.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Duplicate motor_id detected — check wiring and URDF parameters");
            return CallbackReturn::ERROR;
        }
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            if(kd_.at(i)<0 || kp_.at(i)<0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "wrong values for kp and kd check please");
                return CallbackReturn::ERROR;
            }
            if (motor_scale_pos_[i] <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Invalid position scale for joint '%s': %.6f", info_.joints[i].name.c_str(), motor_scale_pos_[i]);
                return CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"), "on_configure OK");

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RobstrideControl::export_state_interface()
    {
        std::vector<hardware_interface::StateInterface> s1;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {

            s1.emplace_back(info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &pos_state_[i]);
            s1.emplace_back(info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);
        }
        return s1;
    }
    std::vector<hardware_interface::CommandInterface> RobstrideControl::export_command_interface()
    {
        std::vector<hardware_interface::CommandInterface> s1;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            s1.emplace_back(info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
        }
        return s1;
    }
    // ─────────────────────────────────────────────────────────────────────────────
    // on_activate
    //   Bring hardware online: open CAN socket, enable motors, set MIT mode.
    //   Return ERROR here to abort activation safely.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {   // set seed values for the states
        for (size_t i = 0; i < pos_state_.size(); ++i)
        {
            pos_state_[i] = 0.0;
            vel_state_[i] = 0.0;
            torque_state_[i] = 0.0;
            pos_cmd_[i] = 0.0;
        }
        if (!can_.init(can_interface_name_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"),
                         "Failed to open CAN interface '%s'", can_interface_name_.c_str());
            return CallbackReturn::ERROR;
        }
        for (size_t i = 0; i < motor_ids_.size();i++)
        {
            uint32_t ext_id = (CommType::ENABLE << 24) | (0xFF << 8) | (motor_ids_[i]);
            can_.write_frame(ext_id,nullptr,0);
        }
        // Drain status frams from Enable before sending mode writes
        {
            struct can_frame dummy;
            while(can_.read_frame(&dummy,5))
            {}
        }
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            uint8_t payload[6];
            pack_u16_le(payload, ParamID::MODE);
            pack_float_le(payload + 2, static_cast<float>(ControlMode::MIT_MODE));
            uint32_t ext = (CommType::WRITE_PARAMETER << 24) | (0XFF << 8) | motor_ids_[i];
            can_.write_frame(ext_id, payload, 6);

        }
        {
            struct can_frame dummy2; while(can_.read_frame(&dummy2,5)){}}
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"), "on_activate Ok");
        return CallbackReturn::SUCCESS;
    }
    // ─────────────────────────────────────────────────────────────────────────────
    // on_deactivate
    //   Gracefully shut down: disable motors first, then close the socket.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        for (size_t i = 0; i < motor_ids_.size();i++)
        {
            uint32_t ext_id = (CommType::DISABLE << 24) | (0xFF << 8) | (motors_ids_[i]);
            can_.write_frame(ext_id, nullptr, 0);
        }
        can_.close();
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"), "on_deactivate OK");
        return CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // write()
    //   Called every control cycle BEFORE read().
    //   Encodes pos_cmd_[i] into a MIT-mode CAN frame and sends it to each motor.
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for (size_t i = 0; i < motor_ids_.size();i++)
        {
            uint16_t torque_u16 = static_cast<uint16_t>(std::max(0, std::min(65535.0, (0.0 / motor_scale_torque_[i]) * 0XFFFF)));
            uint32_t ext_id = (CommType::OPERATION_CONTROL << 24) | (static_cast<uint32_t>(torque_u16) << 8) | motor_ids_[i];
            uint8_t data[8];
            uint16_t pos_u16 = static_cast<uint16_t>(std::max(0, std::min(65535.0, (pos_cmd_[i] / motor_scale_torque[i]) * 0xFFFF)));
            uint16_t vel_u16 = static_cast<uint16_t>(std::max(0,std::min(65535.0,(0.0)));
            uint16_t kp_u16 = static_cast<uint16_t>(std::max(0,std::mins(65535.0,(kp_[i]/motor_scale_kp_[i] +1)*0xFFFF)));
            uint16_t kd_u16 = static_cast<uint16_t>(std::max(0,std::min(65535.0,kd_[i]/motor_scale_kd_[i]+1)*0xFFFF));

            pack_u16_be(data +0,pos_u16);
            pack_u16_be(data+2, pos_u16 );
            pack_u16_be(data +4, kp_u16);
            pack_u16_be(data+6,kd_u16);
            can_.write_frame(ext_id,data,8);
        }
        return hardware_interface::return_type::OK;
    }
    // ─────────────────────────────────────────────────────────────────────────────
    // ─────────────────────────────────────────────────────────────────────────────
    // read()
    //   Called every control cycle AFTER write().
    //   Receives the STATUS frame each motor echoes back and updates state buffers.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::return_type RobstrideControl::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        for (size_t i = 0; i < motor_ids_.size();i++)
        {
            struct can_frame frame;
                if(!can_.read_frame(&frame,5))
                {
                    static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RobStrideSystem"),
                                         steady_clock, 1000,
                                         "Timeout waiting for motor %d — keeping stale state", motor_ids_[i]);
                    continue;
                }
                uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
                uint32_t comm_type = (raw_id >> 24) & 0x1F;
                uint8_t src_id = (raw_id >> 8) & 0xFF;
                if(comm_type!= CommType::OPERATION_STATUS || src_id != motor_ids_[i])
                {
                    RCLCPP_WARN(rclcpp::get_logger("RobStrideSystem"),
                                "Unexpected frame: comm_type=%u src=%u (expected motor %u)",
                                comm_type, src_id, motor_ids_[i]);
                    continue;
                }
                uint16_t pos_raw = (uint16_t(frame.data[0]) << 8) | frame.data[1];
                uint16_t vel_raw = (uint16_t(frame.data[2]) << 8) | frame.data[3];
                uint16_t torque_raw = (uint16_t(frame.data[4]) << 8) | frame.data[5];

                pos_state_[i] = 
        }
    }
}