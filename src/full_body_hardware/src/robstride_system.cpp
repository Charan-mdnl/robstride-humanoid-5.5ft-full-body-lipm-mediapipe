#include "full_body_hardware/robstride_system.hpp"

#include "full_body_hardware/can_interfaces.h"
#include "full_body_hardware/protocol_completed.h"

#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <set>
#include <chrono>

// Plugin registration — ros2_control discovers this class via pluginlib.
// The string must match the <class name="..."> in full_body_hardware_plugins.xml.
PLUGINLIB_EXPORT_CLASS(
    full_body_hardware::RobStrideSystem,
    hardware_interface::SystemInterface)

namespace full_body_hardware
{

static rclcpp::Logger LOG = rclcpp::get_logger("RobStrideSystem");

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────
RobStrideSystem::RobStrideSystem()
    : SystemInterface()
{
    RCLCPP_INFO(LOG, "Constructor called");
}

RobStrideSystem::~RobStrideSystem()
{
    // Safety net: make sure workers are stopped and sockets closed even if
    // on_deactivate was never called (e.g. process teardown).
    run_ = false;
    for (auto & bus : buses_)
    {
        if (bus && bus->worker.joinable())
            bus->worker.join();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// on_init
//   Parse URDF <param> tags, validate configuration, size all vectors and
//   group joints by the CAN bus they live on.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobStrideSystem::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    const size_t n = info_.joints.size();

    // ── Hardware-level params ────────────────────────────────────────────────
    can_interface_name_ = info_.hardware_parameters.count("can_interface")
        ? info_.hardware_parameters.at("can_interface")
        : "can2";
    if (info_.hardware_parameters.count("update_rate"))
    {
        try { update_rate_hz_ = std::stoi(info_.hardware_parameters.at("update_rate")); }
        catch (const std::exception &) { /* keep default */ }
    }
    if (update_rate_hz_ <= 0) update_rate_hz_ = 200;

    // ── Per-joint params, and which bus each joint belongs to ────────────────
    std::vector<std::string> joint_bus(n);  // resolved bus name per joint

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto & joint = info_.joints[i];
        try {
            motor_ids_.push_back(static_cast<uint8_t>(std::stoi(joint.parameters.at("motor_id"))));
        }
        catch (const std::exception &)
        {
            RCLCPP_ERROR(LOG, "Joint '%s' is missing <param name=\"motor_id\">", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        try {
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
                RCLCPP_ERROR(LOG, "Joint '%s' has unknown model '%s'", joint.name.c_str(), model.c_str());
                return CallbackReturn::ERROR;
            }
            double kp_val = joint.parameters.count("kp") ? std::stod(joint.parameters.at("kp")) : 15.00;
            double kd_val = joint.parameters.count("kd") ? std::stod(joint.parameters.at("kd")) : 1.40;
            kp_.push_back(kp_val);
            kd_.push_back(kd_val);
            bool invert_val = joint.parameters.count("invert") && joint.parameters.at("invert") == "true";
            invert_.push_back(invert_val);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(LOG, "Error parsing joint parameters: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // Which bus does this joint live on? Per-joint param wins, else the
        // hardware-level default.
        joint_bus[i] = joint.parameters.count("can_interface")
            ? joint.parameters.at("can_interface")
            : can_interface_name_;
    }

    // ── Group joints into buses (one socket + one thread per unique name) ─────
    for (size_t i = 0; i < n; ++i)
    {
        MotorBus * bus = nullptr;
        for (auto & b : buses_)
            if (b->name == joint_bus[i]) { bus = b.get(); break; }
        if (!bus)
        {
            buses_.push_back(std::make_unique<MotorBus>());
            buses_.back()->name = joint_bus[i];
            bus = buses_.back().get();
        }
        bus->idx.push_back(i);
        id_to_idx_[motor_ids_[i]] = i;
    }

    pos_state_.resize(n, 0.0);
    vel_state_.resize(n, 0.0);
    torque_state_.resize(n, 0.0);
    pos_cmd_.resize(n, 0.0);

    cmd_shared_.resize(n, 0.0);
    st_pos_shared_.resize(n, 0.0);
    st_vel_shared_.resize(n, 0.0);
    st_torque_shared_.resize(n, 0.0);

    std::string bus_summary;
    for (auto & b : buses_)
        bus_summary += " " + b->name + "(" + std::to_string(b->idx.size()) + ")";
    RCLCPP_INFO(LOG, "on_init OK — %zu joints across %zu bus(es):%s @ %d Hz",
                n, buses_.size(), bus_summary.c_str(), update_rate_hz_);
    return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobStrideSystem::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    std::set<uint8_t> unique_ids(motor_ids_.begin(), motor_ids_.end());
    if (unique_ids.size() != motor_ids_.size()) {
        RCLCPP_ERROR(LOG, "Duplicate motor_id detected — check wiring and URDF parameters");
        return CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (kp_[i] < 0 || kd_[i] < 0) {
            RCLCPP_ERROR(LOG, "Invalid gains for joint '%s': kp=%.2f, kd=%.2f",
                         info_.joints[i].name.c_str(), kp_[i], kd_[i]);
            return CallbackReturn::ERROR;
        }
        if (motor_scale_pos_[i] <= 0) {
            RCLCPP_ERROR(LOG, "Invalid position scale for joint '%s': %.6f",
                         info_.joints[i].name.c_str(), motor_scale_pos_[i]);
            return CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(LOG, "on_configure OK");
    return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
RobStrideSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        si.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &pos_state_[i]);
        si.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);
    }
    return si;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
RobStrideSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        ci.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
    }
    return ci;
}

// ─────────────────────────────────────────────────────────────────────────────
// build_mit_frame — encode one joint's MIT operation-control frame
// ─────────────────────────────────────────────────────────────────────────────
void RobStrideSystem::build_mit_frame(
    size_t i, double pos_cmd, uint32_t & ext_id, uint8_t data[8]) const
{
    uint16_t torque_u16 = encode_signed(0.0, motor_scale_torque_[i]);
    ext_id = (CommType::OPERATION_CONTROL << 24)
           | (static_cast<uint32_t>(torque_u16) << 8)
           | motor_ids_[i];
    double cmd = invert_[i] ? -pos_cmd : pos_cmd;
    uint16_t pos_u16 = encode_signed  (cmd,    motor_scale_pos_[i]);
    uint16_t vel_u16 = encode_signed  (0.0,    motor_scale_vel_[i]);
    uint16_t kp_u16  = encode_unsigned(kp_[i], motor_scale_kp_[i]);
    uint16_t kd_u16  = encode_unsigned(kd_[i], motor_scale_kd_[i]);
    pack_u16_be(data + 0, pos_u16);
    pack_u16_be(data + 2, vel_u16);
    pack_u16_be(data + 4, kp_u16);
    pack_u16_be(data + 6, kd_u16);
}

// ─────────────────────────────────────────────────────────────────────────────
// activate_bus — bring one bus online (runs before its worker starts)
// ─────────────────────────────────────────────────────────────────────────────
bool RobStrideSystem::activate_bus(MotorBus & bus)
{
    if (!bus.can.init(bus.name))
    {
        RCLCPP_ERROR(LOG, "Failed to open CAN interface '%s'", bus.name.c_str());
        return false;
    }

    // ── Zero each motor INDIVIDUALLY by its motor_id ─────────────────────────
    //   RobStride SET_ZERO_POSITION requires a 1-byte 0x01 payload (DLC 8);
    //   an empty frame is a no-op on the firmware. Ref: RobStride_Control
    //   cpp/include/motor_control.hpp::set_zero_position().
    for (size_t i : bus.idx)
    {
        uint32_t ext_id = (CommType::SET_ZERO_POSITION << 24) | (0xFF << 8) | motor_ids_[i];
        uint8_t data[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
        bus.can.write_frame(ext_id, data, 8);
        RCLCPP_INFO(LOG, "[%s] SET_ZERO_POSITION → motor %d", bus.name.c_str(), motor_ids_[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Persist the new zero to flash — SAVE_PARAMETERS is essential so the zero
    // survives a power cycle (payload {0x01..0x08}, DLC 8).
    for (size_t i : bus.idx)
    {
        uint32_t ext_id = (CommType::SAVE_PARAMETERS << 24) | (0xFF << 8) | motor_ids_[i];
        uint8_t save_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        bus.can.write_frame(ext_id, save_data, 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    { struct can_frame d; while (bus.can.read_frame(&d, 5)) {} }

    // ── Enable all motors ────────────────────────────────────────────────────
    for (size_t i : bus.idx)
    {
        uint32_t ext_id = (CommType::ENABLE << 24) | (0xFF << 8) | motor_ids_[i];
        bus.can.write_frame(ext_id, nullptr, 0);
    }
    { struct can_frame d; while (bus.can.read_frame(&d, 5)) {} }

    // ── Set MIT mode ─────────────────────────────────────────────────────────
    for (size_t i : bus.idx)
    {
        uint8_t payload[6];
        pack_u16_le(payload,     ParamID::MODE);
        pack_float_le(payload+2, static_cast<float>(ControlMode::MIT_MODE));
        uint32_t ext_id = (CommType::WRITE_PARAMETER << 24) | (0xFF << 8) | motor_ids_[i];
        bus.can.write_frame(ext_id, payload, 6);
    }
    { struct can_frame d; while (bus.can.read_frame(&d, 5)) {} }

    // Seed buffers so controllers start from a defined position.
    for (size_t i : bus.idx)
    {
        pos_state_[i] = 0.0; vel_state_[i] = 0.0; torque_state_[i] = 0.0; pos_cmd_[i] = 0.0;
    }
    { struct can_frame d; while (bus.can.read_frame(&d, 5)) {} }

    // ── Read current positions: send a zero-force MIT frame to each motor and
    //    dispatch the STATUS echoes back by src_id (order-independent). ───────
    for (size_t i : bus.idx)
    {
        uint32_t ext_id; uint8_t data[8];
        build_mit_frame(i, 0.0, ext_id, data);
        // zero-force: override gains with kp=kd=0 for the seeding frame
        pack_u16_be(data + 4, encode_unsigned(0.0, motor_scale_kp_[i]));
        pack_u16_be(data + 6, encode_unsigned(0.0, motor_scale_kd_[i]));
        bus.can.write_frame(ext_id, data, 8);
    }
    size_t expected = bus.idx.size();
    for (size_t got = 0; got < expected; )
    {
        struct can_frame frame;
        if (!bus.can.read_frame(&frame, 20))
        {
            RCLCPP_WARN(LOG, "[%s] Timeout during init read (%zu/%zu) — defaulting missing to 0.0",
                        bus.name.c_str(), got, expected);
            break;
        }
        uint32_t raw_id    = frame.can_id & CAN_EFF_MASK;
        uint32_t comm_type = (raw_id >> 24) & 0x1F;
        uint8_t  src_id    = (raw_id >> 8) & 0xFF;
        auto it = id_to_idx_.find(src_id);
        if (comm_type != CommType::OPERATION_STATUS || it == id_to_idx_.end())
            continue;  // stray frame — ignore, don't count
        size_t i = it->second;
        uint16_t pos_raw = (uint16_t(frame.data[0]) << 8) | frame.data[1];
        uint16_t vel_raw = (uint16_t(frame.data[2]) << 8) | frame.data[3];
        double sign = invert_[i] ? -1.0 : 1.0;
        pos_state_[i] = sign * decode_signed(pos_raw, motor_scale_pos_[i]);
        vel_state_[i] = sign * decode_signed(vel_raw, motor_scale_vel_[i]);
        pos_cmd_[i]   = pos_state_[i];  // hold current position on first write
        ++got;
        RCLCPP_INFO(LOG, "[%s] Motor %d initial position: %.4f rad",
                    bus.name.c_str(), motor_ids_[i], pos_state_[i]);
    }

    // Publish the seeded state into the shared buffers so read() sees it
    // immediately, before the worker's first cycle.
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        for (size_t i : bus.idx)
        {
            cmd_shared_[i]       = pos_cmd_[i];
            st_pos_shared_[i]    = pos_state_[i];
            st_vel_shared_[i]    = vel_state_[i];
            st_torque_shared_[i] = torque_state_[i];
        }
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate
//   Bring every bus online, then launch one worker thread per bus.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobStrideSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    for (auto & bus : buses_)
    {
        if (!activate_bus(*bus))
            return CallbackReturn::ERROR;
    }

    run_ = true;
    for (auto & bus : buses_)
        bus->worker = std::thread(&RobStrideSystem::bus_loop, this, std::ref(*bus));

    RCLCPP_INFO(LOG, "on_activate OK — %zu worker thread(s) running", buses_.size());
    return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// bus_loop — the per-bus worker: write MIT commands, read & dispatch status.
//   Owns bus.can exclusively; the control thread never touches the socket.
// ─────────────────────────────────────────────────────────────────────────────
void RobStrideSystem::bus_loop(MotorBus & bus)
{
    const auto period = std::chrono::microseconds(1000000 / update_rate_hz_);
    auto next = std::chrono::steady_clock::now();

    std::vector<double> local_cmd(bus.idx.size());

    while (run_.load(std::memory_order_relaxed))
    {
        // 1. Snapshot the latest commands for this bus's motors.
        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            for (size_t k = 0; k < bus.idx.size(); ++k)
                local_cmd[k] = cmd_shared_[bus.idx[k]];
        }

        // 2. Send an OPERATION_CONTROL (MIT) frame to each motor.
        for (size_t k = 0; k < bus.idx.size(); ++k)
        {
            uint32_t ext_id; uint8_t data[8];
            build_mit_frame(bus.idx[k], local_cmd[k], ext_id, data);
            bus.can.write_frame(ext_id, data, 8);
        }

        // 3. Read the STATUS echoes and dispatch each by src_id (order-free).
        size_t expected = bus.idx.size();
        for (size_t got = 0; got < expected; )
        {
            struct can_frame frame;
            if (!bus.can.read_frame(&frame, 5))
            {
                static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
                RCLCPP_WARN_THROTTLE(LOG, steady_clock, 1000,
                    "[%s] Timeout — %zu/%zu motors responded, keeping stale state",
                    bus.name.c_str(), got, expected);
                break;
            }
            uint32_t raw_id    = frame.can_id & CAN_EFF_MASK;
            uint32_t comm_type = (raw_id >> 24) & 0x1F;
            uint8_t  src_id    = (raw_id >> 8) & 0xFF;
            auto it = id_to_idx_.find(src_id);
            if (comm_type != CommType::OPERATION_STATUS || it == id_to_idx_.end())
                continue;  // stray/unknown frame — ignore, don't count
            size_t i = it->second;
            uint16_t pos_raw    = (uint16_t(frame.data[0]) << 8) | frame.data[1];
            uint16_t vel_raw    = (uint16_t(frame.data[2]) << 8) | frame.data[3];
            uint16_t torque_raw = (uint16_t(frame.data[4]) << 8) | frame.data[5];
            double sign = invert_[i] ? -1.0 : 1.0;
            double p = sign * decode_signed(pos_raw,    motor_scale_pos_[i]);
            double v = sign * decode_signed(vel_raw,    motor_scale_vel_[i]);
            double t = sign * decode_signed(torque_raw, motor_scale_torque_[i]);
            std::lock_guard<std::mutex> lk(data_mtx_);
            st_pos_shared_[i]    = p;
            st_vel_shared_[i]    = v;
            st_torque_shared_[i] = t;
            ++got;
        }

        // 4. Pace the loop to the target rate (skip ahead if we fell behind).
        next += period;
        auto now = std::chrono::steady_clock::now();
        if (next > now)
            std::this_thread::sleep_until(next);
        else
            next = now;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate
//   Stop worker threads, disable motors, close sockets.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn RobStrideSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    run_ = false;
    for (auto & bus : buses_)
        if (bus->worker.joinable())
            bus->worker.join();

    // Now the control thread owns each socket again — safe to disable & close.
    for (auto & bus : buses_)
    {
        for (size_t i : bus->idx)
        {
            uint32_t ext_id = (CommType::DISABLE << 24) | (0xFF << 8) | motor_ids_[i];
            bus->can.write_frame(ext_id, nullptr, 0);
        }
        bus->can.close();
    }

    RCLCPP_INFO(LOG, "on_deactivate OK");
    return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// write() — copy controller commands into the shared buffer for the workers.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type RobStrideSystem::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lk(data_mtx_);
    for (size_t i = 0; i < pos_cmd_.size(); ++i)
        cmd_shared_[i] = pos_cmd_[i];
    return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// read() — copy the workers' latest state into the controller-facing buffers.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type RobStrideSystem::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lk(data_mtx_);
    for (size_t i = 0; i < pos_state_.size(); ++i)
    {
        pos_state_[i]    = st_pos_shared_[i];
        vel_state_[i]    = st_vel_shared_[i];
        torque_state_[i] = st_torque_shared_[i];
    }
    return hardware_interface::return_type::OK;
}

}  // namespace full_body_hardware
