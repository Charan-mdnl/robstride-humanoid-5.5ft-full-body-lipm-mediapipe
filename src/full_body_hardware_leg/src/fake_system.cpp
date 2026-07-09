#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
// TODO: add these includes once the private members are added below
// #include "hardware_interface/handle.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "pluginlib/class_list_macros.hpp"

using namespace std::chrono_literals;

namespace full_body_hardware
{
      class FakeSystem : public hardware_interface::SystemInterface
      {
        public:
        FakeSystem(): SystemInterface()
        {
          RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem constructor called");
        }
        protected:
        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
        {
            if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
            //
         

           for (size_t i = 0; i < info.joints.size(); i++)
           {
            /* code */
          //  info.joints.size(); // just to avoid "unused variable" warning until the TODO is implemented
            RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem on_init called with %zu joints", info.joints.size());
           }
           pos_state_.resize(info.joints.size(), 0.0);
           vel_state_.resize(info.joints.size(), 0.0);
           pos_cmd_.resize(info.joints.size(), 0.0);           
         

            RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem on_init called");
            return CallbackReturn::SUCCESS;
        }

        
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override
            {
                std::vector<hardware_interface::StateInterface> state_interfaces;
                for (size_t i = 0; i<info_.joints.size(); i++)
                {
                        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[i]);
                        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);    

                    }
                    return state_interfaces;
            }
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
            {
                std::vector<hardware_interface::CommandInterface> command_interfaces;
                for (size_t i = 0; i<info_.joints.size(); i++)
                {
                        command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
                    }
                    return command_interfaces;
            }
        
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem on_configure called");
            return CallbackReturn::SUCCESS;
        }   
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem on_cleanup called");
            return CallbackReturn::SUCCESS;
        }
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
        {
            RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem on_shutdown called");
            return CallbackReturn::SUCCESS; 
        }
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
        {
            // TODO: seed states and commands from initial_value set in the xacro (all 0.0).
            //   for (std::size_t i = 0; i < info_.joints.size(); ++i) {
            //       pos_state_[i] = 0.0;
            //       vel_state_[i] = 0.0;
            //       pos_cmd_[i]   = 0.0;
            //   }
            //
            // For real arm hardware you would ALSO open the communication interface here
            // (e.g. connect to the motor driver, enable torque) before returning SUCCESS.
            for(size_t i = 0; i<info_.joints.size(); i++)
            {
                pos_state_[i] = 0.0;
                vel_state_[i] = 0.0;
                pos_cmd_[i]   = 0.0;                
            }
            RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem on_activate called");
            return CallbackReturn::SUCCESS;
        }     
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
        {
            // For real hardware: disable torque / close comms here.
            RCLCPP_INFO(rclcpp::get_logger("FakeSystem"), "FakeSystem on_deactivate called");
            return CallbackReturn::SUCCESS;
        }
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &)override
        {
            for (size_t i = 0; i < pos_state_.size(); i++)
            {
                /* code */
                pos_state_[i] = pos_cmd_[i]; // perfect tracking
                vel_state_[i] = 0.0;
            }
                return hardware_interface::return_type::OK;
                }
                hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
                {
                    return hardware_interface::return_type::OK;
                }

                // TODO: implement read() — must be overridden, currently missing.
                //   This is the core of the fake plugin: copy commands → states every cycle.
                //   For ALL joints (left arm, right arm, legs, torso) do:
                //
                // hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
                // {
                //     for (std::size_t i = 0; i < pos_state_.size(); ++i) {
                //         pos_state_[i] = pos_cmd_[i];   // instant perfect tracking
                //         vel_state_[i] = 0.0;
                //     }
                //     return hardware_interface::return_type::OK;
                // }
                //
                // For real arm hardware: replace pos_state_[i] = pos_cmd_[i] with
                //   pos_state_[i] = read_encoder(joint_id[i]);

                // TODO: implement write() — must be overridden, currently missing.
                //   Fake: no-op.  Real hardware: send pos_cmd_[i] to each motor driver.
                //
                // hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
                // {
                //     return hardware_interface::return_type::OK;
                // }

            private:
                // TODO: declare storage vectors — one entry per joint in info_.joints.

                std::vector<double> pos_state_; // position feedback (read by controllers)
                std::vector<double> vel_state_; // velocity feedback (read by controllers)
                std::vector<double> pos_cmd_;   // position command  (written by controllers)
                                                // For real hardware you would also store per-joint identifiers, e.g.:
                                                // std::vector<int>    joint_can_id_;   // CAN bus IDs for left arm: [10,11,12,13]
                                                //                                      //              for right arm: [20,21,22,23]
    };
}

// TODO: uncomment once pluginlib/class_list_macros.hpp is included above
PLUGINLIB_EXPORT_CLASS(full_body_hardware::FakeSystem, hardware_interface::SystemInterface)
