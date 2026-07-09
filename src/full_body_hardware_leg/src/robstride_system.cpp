#include "full_body_hardware/robstride_system.hpp"

#include "full_body_hardware/can_interfaces.h"
#include "full_body_hardware/protocol_completed.h"

#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <set>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <memory>
// Plugin registration — ros2_control discovers this class via pluginlib.
// The string must match the <class name="..."> in full_body_hardware_plugins.xml.
PLUGINLIB_EXPORT_CLASS(
    full_body_hardware::RobStrideSystem,
    hardware_interface::SystemInterface)

namespace full_body_hardware
{

    // ─────────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────────
    RobStrideSystem::RobStrideSystem()
        : SystemInterface()
    {
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"), "Constructor called");
    }
RobStrideSystem::~RobStrideSystem()
{
    run_ = false;
    for(auto & bus:buses_)
    {
        if(bus->worker.joinable())
            bus->worker.join();
    }
}
    // ─────────────────────────────────────────────────────────────────────────────
    // on_init
    //   Called once when the plugin is loaded.
    //   Parse URDF <param> tags, validate configuration, size all vectors.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn RobStrideSystem::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        const size_t n = info_.joints.size();

        // ── Hardware-level params ────────────────────────────────────────────────

        can_interface_name_ = info_.hardware_parameters.count("can_interface")
                                  ? info_.hardware_parameters.at("can_interface")
                                  : "can0";
        // ── Per-joint params ─────────────────────────────────────────────────────

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            const auto &joint = info_.joints[i];
            try
            {
                motor_ids_.push_back(static_cast<uint8_t>(std::stoi(joint.parameters.at("motor_id"))));
            }
            catch (const std::exception &)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Joint '%s' is missing <param name=\"motor_id\">", joint.name.c_str());
                return CallbackReturn::ERROR;
            }
            try
            {
                std::string model = (joint.parameters.at("motor_model"));
                motor_models_.push_back(model);
                if (model == "rs03")
                {
                    motor_scale_pos_.push_back(ModelScale::RS03::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS03::VELOCITY);
                    motor_scale_torque_.push_back(ModelScale::RS03::TORQUE);
                    motor_scale_kp_.push_back(ModelScale::RS03::KP);
                    motor_scale_kd_.push_back(ModelScale::RS03::KD);
                }
                else if (model == "rs00")
                {
                    motor_scale_pos_.push_back(ModelScale::RS00::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS00::VELOCITY);
                    motor_scale_torque_.push_back(ModelScale::RS00::TORQUE);
                    motor_scale_kp_.push_back(ModelScale::RS00::KP);
                    motor_scale_kd_.push_back(ModelScale::RS00::KD);
                }
                else if (model == "rs02")
                {
                    motor_scale_pos_.push_back(ModelScale::RS02::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS02::VELOCITY);
                    motor_scale_torque_.push_back(ModelScale::RS02::TORQUE);
                    motor_scale_kp_.push_back(ModelScale::RS02::KP);
                    motor_scale_kd_.push_back(ModelScale::RS02::KD);
                }
                else if (model == "rs04")
                {
                    motor_scale_pos_.push_back(ModelScale::RS04::POSITION);
                    motor_scale_vel_.push_back(ModelScale::RS04::VELOCITY);
                    motor_scale_torque_.push_back(ModelScale::RS04::TORQUE);
                    motor_scale_kp_.push_back(ModelScale::RS04::KP);
                    motor_scale_kd_.push_back(ModelScale::RS04::KD);
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Joint '%s' has unknown model '%s'", joint.name.c_str(), model.c_str());
                    return CallbackReturn::ERROR;
                }
                mass_.push_back(joint.parameters.count("mass")         ? std::stod(joint.parameters.at("mass"))         : 1.0);
                length_.push_back(joint.parameters.count("length")       ? std::stod(joint.parameters.at("length"))       : 0.1);
                offset_.push_back(joint.parameters.count("theta_offset") ? std::stod(joint.parameters.at("theta_offset")) : 0.0);
                inertia_.push_back(joint.parameters.count("inertia")     ? std::stod(joint.parameters.at("inertia"))      : 3.5);
                e_nominal_.push_back(joint.parameters.count("e_nominal") ? std::stod(joint.parameters.at("e_nominal"))    : 0.1);
                kd_max_.push_back(joint.parameters.count("kd_max")       ? std::stod(joint.parameters.at("kd_max"))       : 100.0);
                bool invert_val = joint.parameters.count("invert") && joint.parameters.at("invert") == "true";
                invert_.push_back(invert_val);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Error parsing joint parameters: %s", e.what());
                return CallbackReturn::ERROR;
            }
        }

        pos_state_.resize(n, 0.0);
        vel_state_.resize(n, 0.0);
        torque_state_.resize(n, 0.0);
        pos_cmd_.resize(n, 0.0);
    for(size_t i=0;i<n;i++)
    {   
        std::string bus_name = info_.joints[i].parameters.count("can_interface")? info_.joints[i].parameters.at("can_interface"):can_interface_name_;
        MotorBus *b =nullptr;
        for(auto &p:buses_)  if(p->name==bus_name)
        {
            b = p.get();
            break;
        }
        if(!b)
        {
            buses_.push_back(std::make_unique<MotorBus>());
            buses_.back()->name = bus_name;
            b = buses_.back().get();

        }
        b->idx.push_back(i);
        id_to_idx[motor_ids_[i]] = i;
    }
    cmd_shared_.resize(n,0.0);
    st_pos_shared_.resize(n,0.0);
    st_vel_shared_.resize(n,0.0);
    st_torque_shared_.resize(n,0.0);
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"),
                    "on_init OK — %zu joints, CAN interface: %s", n, can_interface_name_.c_str());
        return CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // on_configure
    //   Validate configuration before the hardware is activated.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn RobStrideSystem::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::set<uint8_t> unique_ids(motor_ids_.begin(), motor_ids_.end());
        if (unique_ids.size() != motor_ids_.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Duplicate motor_id detected — check wiring and URDF parameters");
            return CallbackReturn::ERROR;
        }
        for (size_t i = 0; i < motor_ids_.size(); ++i)
        {
            if (inertia_[i] <= 0 || e_nominal_[i] <= 0 || kd_max_[i] <= 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"), "Invalid feedforward params for joint '%s': inertia=%.2f e_nominal=%.4f kd_max=%.2f", info_.joints[i].name.c_str(), inertia_[i], e_nominal_[i], kd_max_[i]);
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

    // ─────────────────────────────────────────────────────────────────────────────
    // export_state_interfaces
    //   Declare which physical quantities controllers can READ.
    // ─────────────────────────────────────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface>
    RobStrideSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> si;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            si.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION, &pos_state_[i]);
            si.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);

            // TODO (Step 2e): Optionally expose torque feedback.
            //   Uncomment to allow torque controllers to read real torque:
            //   si.emplace_back(info_.joints[i].name,
            //       hardware_interface::HW_IF_EFFORT, &torque_state_[i]);
        }
        return si;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // export_command_interfaces
    //   Declare which quantities controllers can WRITE.
    // ─────────────────────────────────────────────────────────────────────────────
    std::vector<hardware_interface::CommandInterface>
    RobStrideSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> ci;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            ci.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
        }
        return ci;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // on_activate
    //   Bring hardware online: open CAN socket, enable motors, set MIT mode.
    //   Return ERROR here to abort activation safely.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn RobStrideSystem::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for(auto &bus:buses_)
            activate_bus(*bus);
        run_ = true;
        for(auto &bus:buses_){
            bus->worker = std::thread(&RobStrideSystem::bus_loop,this,std::ref(*bus));
        }
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"),
                    "on_activate OK — %zu bus threads", buses_.size());
        return CallbackReturn::SUCCESS;
    }
void RobStrideSystem::activate_bus(MotorBus &bus)
{
    if(!bus.can.init(bus.name))
    {        RCLCPP_ERROR(rclcpp::get_logger("RobStrideSystem"),
                     "Failed to open CAN '%s'", bus.name.c_str());
        return;}
        // set zero
        for(size_t i:bus.idx)
        {
            uint32_t id=(CommType::SET_ZERO_POSITION<<24)|(0xFF<<8)|motor_ids_[i];
            uint8_t d[8]={0x01,0,0,0,0,0,0,0};
            bus.can.write_frame(id,d,8);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
  for (size_t i : bus.idx) {
        uint32_t id=(CommType::SAVE_PARAMETERS<<24)|(0xFF<<8)|motor_ids_[i];
        uint8_t d[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
        bus.can.write_frame(id,d,8);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    { struct can_frame f; while (bus.can.read_frame(&f,5)){} }

    for(size_t i:bus.idx)
    {
       uint8_t p[6]; pack_u16_le(p,ParamID::MODE);
        pack_float_le(p+2,static_cast<float>(ControlMode::MIT_MODE));
        uint32_t id=(CommType::WRITE_PARAMETER<<24)|(0xFF<<8)|motor_ids_[i];
        bus.can.write_frame(id,p,6);
    }
        { struct can_frame f; while (bus.can.read_frame(&f,5)){} }
// ── read initial positions (zero-force MIT), dispatch by src_id ──
    bus.local_pos.assign(bus.idx.size(),0.0);
    for (size_t i : bus.idx) {
        uint32_t id=(CommType::OPERATION_CONTROL<<24)
                   |((uint32_t)encode_signed(0.0,motor_scale_torque_[i])<<8)|motor_ids_[i];
        uint8_t d[8];
        pack_u16_be(d+0, encode_signed(0.0,motor_scale_pos_[i]));
        pack_u16_be(d+2, encode_signed(0.0,motor_scale_vel_[i]));
        pack_u16_be(d+4, encode_unsigned(0.0,motor_scale_kp_[i]));
        pack_u16_be(d+6, encode_unsigned(0.0,motor_scale_kd_[i]));
        bus.can.write_frame(id,d,8);
    }
    for (size_t got=0; got<bus.idx.size(); ) {
        struct can_frame f;
        if (!bus.can.read_frame(&f,20)) break;
        uint32_t raw=f.can_id&CAN_EFF_MASK;
        uint8_t src=(raw>>8)&0xFF;
        auto it=id_to_idx.find(src);
        if (((raw>>24)&0x1F)!=CommType::OPERATION_STATUS || it==id_to_idx.end()) continue;
        size_t i=it->second;
        uint16_t pr=(uint16_t(f.data[0])<<8)|f.data[1];
        double sign=invert_[i]?-1.0:1.0;
        double p=sign*decode_signed(pr,motor_scale_pos_[i]);
        pos_state_[i]=p; pos_cmd_[i]=p;
        for (size_t k=0;k<bus.idx.size();++k) if (bus.idx[k]==i) bus.local_pos[k]=p;
        st_pos_shared_[i]=p; cmd_shared_[i]=p;
        ++got;
    }
}
void RobStrideSystem::bus_loop(MotorBus & bus)
{
    const auto period = std::chrono::microseconds(1000000/update_rate_hz_);
    auto next = std::chrono::steady_clock::now();
    std::vector<double> cmd(bus.idx.size());

    while (run_.load(std::memory_order_relaxed))
    {
        // 1. snapshot commands
        { std::lock_guard<std::mutex> lk(data_mtx_);
          for (size_t k=0;k<bus.idx.size();++k) cmd[k]=cmd_shared_[bus.idx[k]]; }

        // 2. gravity comp + send
        for (size_t k=0;k<bus.idx.size();++k) {
            size_t i=bus.idx[k];
            double theta=bus.local_pos[k];
            double tau_ff=mass_[i]*9.81*length_[i]*std::sin(theta+offset_[i]);
            double kp=std::max(kp_min_, std::abs(tau_ff)/e_nominal_[i]);
            double kd=std::min(kd_max_[i], 2.0*std::sqrt(kp*inertia_[i]));
            double cmd_tau=invert_[i]?-tau_ff:tau_ff;
            uint32_t id=(CommType::OPERATION_CONTROL<<24)
                       |((uint32_t)encode_signed(cmd_tau,motor_scale_torque_[i])<<8)|motor_ids_[i];
            uint8_t d[8];
            double c=invert_[i]?-cmd[k]:cmd[k];
            pack_u16_be(d+0, encode_signed(c,motor_scale_pos_[i]));
            pack_u16_be(d+2, encode_signed(0.0,motor_scale_vel_[i]));
            pack_u16_be(d+4, encode_unsigned(kp,motor_scale_kp_[i]));
            pack_u16_be(d+6, encode_unsigned(kd,motor_scale_kd_[i]));
            bus.can.write_frame(id,d,8);
        }

        // 3. read replies, dispatch by src_id
        for (size_t got=0; got<bus.idx.size(); ) {
            struct can_frame f;
            if (!bus.can.read_frame(&f,5)) break;
            uint32_t raw=f.can_id&CAN_EFF_MASK;
            uint8_t src=(raw>>8)&0xFF;
            auto it=id_to_idx.find(src);
            if (((raw>>24)&0x1F)!=CommType::OPERATION_STATUS || it==id_to_idx.end()) continue;
            size_t i=it->second;
            uint16_t pr=(uint16_t(f.data[0])<<8)|f.data[1];
            uint16_t vr=(uint16_t(f.data[2])<<8)|f.data[3];
            uint16_t tr=(uint16_t(f.data[4])<<8)|f.data[5];
            double sign=invert_[i]?-1.0:1.0;
            double p=sign*decode_signed(pr,motor_scale_pos_[i]);
            double v=sign*decode_signed(vr,motor_scale_vel_[i]);
            double t=sign*decode_signed(tr,motor_scale_torque_[i]);
            for (size_t k=0;k<bus.idx.size();++k) if (bus.idx[k]==i) bus.local_pos[k]=p;
            { std::lock_guard<std::mutex> lk(data_mtx_);
              st_pos_shared_[i]=p; st_vel_shared_[i]=v; st_torque_shared_[i]=t; }
            ++got;
        }

        // 4. pace
        next+=period;
        auto now=std::chrono::steady_clock::now();
        if (next>now) std::this_thread::sleep_until(next); else next=now;
    }
}

    // ─────────────────────────────────────────────────────────────────────────────
    // on_deactivate
    //   Gracefully shut down: disable motors first, then close the socket.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn RobStrideSystem::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        run_ = false;
        for (auto & bus : buses_) if (bus->worker.joinable()) bus->worker.join();
        for (auto & bus : buses_) {
            for (size_t i : bus->idx) {
                uint32_t id=(CommType::DISABLE<<24)|(0xFF<<8)|motor_ids_[i];
                bus->can.write_frame(id,nullptr,0);
            }
            bus->can.close();
        }
        RCLCPP_INFO(rclcpp::get_logger("RobStrideSystem"), "on_deactivate OK");
        return CallbackReturn::SUCCESS;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // write()
    //   Called every control cycle BEFORE read().
    //   Encodes pos_cmd_[i] into a MIT-mode CAN frame and sends it to each motor.
    // ─────────────────────────────────────────────────────────────────────────────
    hardware_interface::return_type RobStrideSystem::write(const rclcpp::Time&, const rclcpp::Duration&) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    for (size_t i=0;i<pos_cmd_.size();++i) cmd_shared_[i]=pos_cmd_[i];
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type RobStrideSystem::read(const rclcpp::Time&, const rclcpp::Duration&) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    for (size_t i=0;i<pos_state_.size();++i) {
        pos_state_[i]=st_pos_shared_[i]; vel_state_[i]=st_vel_shared_[i]; torque_state_[i]=st_torque_shared_[i];
    }
    return hardware_interface::return_type::OK;
}

} // namespace full_body_hardware