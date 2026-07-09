#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include "../include/can_interfaces.h" 
#include "../include/protocol_completed.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <deque>
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>


std::atomic<bool> running(true);
// --- Motor and protocol definitions ---
#define MOTOR_ID_DEFAULT 11 // default motor ID
#define CAN_INTERFACE "can0"
#define HOST_ID 0xFF
// Communication types
#define COMM_OPERATION_CONTROL 1
#define COMM_ENABLE 3
#define COMM_WRITE_PARAMETER 18
// Parameter IDs
#define PARAM_MODE 0x7005
#define PARAM_VELOCITY_LIMIT 0x7017 
#define PARAM_TORQUE_LIMIT 0x700B 
#define kp_1 4.5
#define kd_1 1.7
// --- Function prototypes ---

bool enable_motor(int s,int motor_id);
bool set_mode_raw(int s,int motor_id,int8_t mode);
bool write_limit(int s,int motor_id,uint16_t param_id,float limit);
bool write_operation_frame(int s,int motor_id,double pos,double kp_val,double kd_val);
bool read_operation_frame(int s);
int init_can(const char* interface);
void signal_handler(int signum);

struct MotorControl {
    int motor_id;
    std::atomic<double> target_position;
    std::atomic<int16_t> v_des;
    std::atomic<double> kp;
    std::atomic<double> kd;
    std::atomic<double> fb_position{0.0};
    std::atomic<double> fb_velocity{0.0};
    std::atomic<double> fb_torque{0.0};
    std::atomic<double> zero_offset{0.0};

    MotorControl(int id, double tp, int16_t vd, double k_p, double k_d)
        : motor_id(id), target_position(tp), v_des(vd), kp(k_p), kd(k_d) {}

    MotorControl(const MotorControl&) = delete;
    MotorControl& operator=(const MotorControl&) = delete;
    MotorControl(MotorControl&&) = delete;
    MotorControl& operator=(MotorControl&&) = delete;
};
// --- CAN communication helper functions ---
/**
 * @brief Send a CAN frame
 */
bool send_frame(int s,uint32_t ext_id,const uint8_t* data,uint8_t dlc)
{  struct can_frame frame;
    frame.can_id = ext_id | CAN_EFF_FLAG; // Extended ID
    frame.can_dlc = dlc;
    if (data && dlc > 0) {
        memcpy(frame.data, data, dlc);
    }
    if (write(s, &frame, sizeof(struct can_frame)) < 0) {
        perror("write");
        return false;
    }
    return true;
}

bool read_frame(int s, struct can_frame* frame,int timeout_ms)
{   fd_set read_fds;
    struct timeval timeout;
    FD_ZERO(&read_fds);
    FD_SET(s, &read_fds);
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    int ret = select(s + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(s, &read_fds)) {
        if (read(s, frame, sizeof(struct can_frame)) < 0) {
            perror("read");
            return false;
        }
        return true;
    }
    return false; // timeout or error
}
// --- Protocol-specific functions ---
bool enable_motor(int s,int motor_id)
{   uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}
bool set_mode_raw(int s,int motor_id,int8_t mode)
{   uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], PARAM_MODE); // param ID
    data[4] = (uint8_t)mode;           // value (int8)
    return send_frame(s, ext_id, data, 8);
}
bool write_limit(int s,int motor_id,uint16_t param_id,float limit)
{   uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], param_id); // param ID
    pack_float_le(&data[4], limit);  // value (float)
    return send_frame(s, ext_id, data, 8);
}
bool write_operation_frame(int s,int motor_id,double pos,double kp_val,double kd_val)
{   // Clamp inputs to valid ranges
    pos    = std::max(-ModelScale::RS03::POSITION, std::min(ModelScale::RS03::POSITION, pos));
    kp_val = std::max(0.0, std::min(ModelScale::RS03::KP, kp_val));
    kd_val = std::max(0.0, std::min(ModelScale::RS03::KD, kd_val));
    // Signed: map [-max, +max] → [0, 0x7FFF*2]  (0 = 0x7FFF)
    // Unsigned: map [0, max] → [0, 0xFFFF]
    uint16_t pos_u16    = (uint16_t)(((pos    / ModelScale::RS03::POSITION) + 1.0) * 0x7FFF);
    uint16_t vel_u16    = 0x7FFF; // 0 velocity
    uint16_t kp_u16     = (uint16_t)((kp_val / ModelScale::RS03::KP) * 0xFFFF);
    uint16_t kd_u16     = (uint16_t)((kd_val / ModelScale::RS03::KD) * 0xFFFF);
    uint16_t torque_u16 = 0x7FFF; // 0 torque_ff
    uint8_t data[8];
    pack_u16_be(&data[0], pos_u16);
    pack_u16_be(&data[2], vel_u16);
    pack_u16_be(&data[4], kp_u16);
    pack_u16_be(&data[6], kd_u16);
    // 2. Build CAN ID
    uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
    // 3. Send
    return send_frame(s, ext_id, data, 8);
}   
bool read_operation_frame(int s)
{   struct can_frame frame;
    if (read_frame(s, &frame, 100)) {
        if (!(frame.can_id & CAN_EFF_FLAG)) return false; // only extended frames
        uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
        if (comm_type == COMM_OPERATION_CONTROL) { // status packet
            return true;
        }
    }
    return false;
}
int init_can(const char* interface)
{   int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return -1;
    }
    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface);    
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return -1;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }
    return s;
}

// Decodes one OPERATION_STATUS frame into the matching motor's feedback atomics.
void parse_status_frame(const struct can_frame& frame, std::deque<MotorControl>& motors)
{
    int src_id = (frame.can_id >> 8) & 0xFF;
    for (auto& m : motors) {
        if (m.motor_id != src_id) continue;
        uint16_t pos_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
        uint16_t vel_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
        uint16_t tor_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
        // Map 0-65535 back to signed physical range
        m.fb_position = (pos_raw / 65535.0 * 2.0 - 1.0) * ModelScale::RS03::POSITION;
        m.fb_velocity = (vel_raw / 65535.0 * 2.0 - 1.0) * ModelScale::RS03::VELOCITY;
        m.fb_torque   = (tor_raw / 65535.0 * 2.0 - 1.0) * ModelScale::RS03::TORQUE;
        return;
    }
}

// Drains the CAN socket and updates feedback atomics for each motor.
void reader_thread(int s, std::deque<MotorControl>& motors)
{
    struct can_frame frame;
    while (running) {
        if (!read_frame(s, &frame, 100)) continue;          // 100ms timeout — re-checks `running`
        if (!(frame.can_id & CAN_EFF_FLAG)) continue;       // skip standard frames
        uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
        if (comm_type != CommType::OPERATION_STATUS) continue;
        parse_status_frame(frame, motors);
    }
}

// Sends a control frame to every motor at 50 Hz.
void writer_thread(int s, std::deque<MotorControl>& motors)
{
    while (running) {
        for (auto& m : motors) {
            write_operation_frame(s, m.motor_id,
                                  m.target_position.load() + m.zero_offset.load(),
                                  m.kp.load(),
                                  m.kd.load());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void signal_handler(int signum)
{
    (void)signum;

    std::cout << "\n Exit signal caught..." << std::endl;
    running = false;
}

int main(int argc, char* argv[])
{
    // TODO 1: Replace single MotorControl with std::vector<MotorControl>
    //         Loop over argv[1..argc-1], push one MotorControl per ID
    //         If no args given, default to a single motor with MOTOR_ID_DEFAULT

    std::deque<MotorControl> motors;
    if (argc > 1 && std::string(argv[1]) == "--range" && argc > 3) {
        int start = std::atoi(argv[2]);
        int end   = std::atoi(argv[3]);
        for (int motor_id = start; motor_id <= end; motor_id++) {
            motors.emplace_back(motor_id, 0.0, static_cast<int16_t>(0), kp_1,kd_1);
        }
        std::cout << "Motor IDs: " << start << " to " << end << std::endl;
    } else if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            int id = std::atoi(argv[i]);
            motors.emplace_back(id, 0.0, static_cast<int16_t>(0), kp_1, kd_1);
        }
    } else {
            // motors.emplace_back(MOTOR_ID_DEFAULT, 0.0, static_cast<int16_t>(0), 5.0, 1.7);
        std::cout << " kindly provide the motor id " << MOTOR_ID_DEFAULT << std::endl;
    }

    // TODO 2: Register signal handlers for SIGINT and SIGTERM
    //         Handler should set running = false
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // TODO 3: Call init_can("can0"), return 1 on failure
   int s = init_can(CAN_INTERFACE);
   if(s<0){
       std::cerr<<" Failed to open CAN interface "<<CAN_INTERFACE<<std::endl;
       return 1;
   }
   std::cout<<" CAN bus "<<CAN_INTERFACE<<" connected"<<std::endl;
    // TODO 4: Enable all motors — loop over the vector
    //         Sleep ~50ms between each to stagger CAN replies
    //         Then sleep 500ms for all motors to wake
    for (const auto& motor : motors) {
        if (enable_motor(s, motor.motor_id)) {
            std::cout << " Enabled motor ID: " << motor.motor_id << std::endl;
        } else {
            std::cerr << "Failed to enable motor ID: " << motor.motor_id << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // TODO 5: Set MIT mode for all motors — loop over the vector
    for (const auto &motor:motors)
    {
         if(set_mode_raw(s,motor.motor_id,ControlMode::MIT_MODE))
         {
                std::cout<<" Set motor ID "<<motor.motor_id<<" to MIT mode"<<std::endl;
         }
         else
         {
                std::cerr<<"Failed to set motor ID "<<motor.motor_id<<" to MIT mode"<<std::endl;
         }
    }
    
    // Start reader only — no writing yet, just observe current positions
    std::thread reader(reader_thread, s, std::ref(motors));

    std::cout << "\nReading motor positions... press 'z' to set zero, Enter to start control, 'q' to quit.\n";
    {
        // Probe thread: sends kp=0, kd=0 frames so the motor sends status frames back.
        // Zero torque is applied — motor is free to be moved by hand.
        std::atomic<bool> probing(true);
        std::thread probe([&]() {
            while (probing && running) {
                for (auto& m : motors)
                    write_operation_frame(s, m.motor_id,
                                          m.fb_position.load(), // hold reported pos = 0 error
                                          0.0, 0.0);            // kp=0, kd=0 → zero torque
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });

        std::string dummy;
        while (running) {
            std::cout << "\n--- Current Motor Positions ---\n";
            for (size_t i = 0; i < motors.size(); i++) {
                std::cout << "[" << i << "] ID:" << motors[i].motor_id
                          << "  pos=" << (motors[i].fb_position.load() - motors[i].zero_offset.load()) << " rad"
                          << "  vel=" << motors[i].fb_velocity.load() << " rad/s"
                          << "  tor=" << motors[i].fb_torque.load()   << " Nm\n";
            }
            std::cout << "[z] set zero  [Enter] start control  [q] quit> " << std::flush;

            fd_set fds;
            struct timeval tv{0, 500000};
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                std::getline(std::cin, dummy);
                if (dummy == "q") { running = false; break; }
                else if (dummy == "z") {
                    for (auto& m : motors)
                        m.zero_offset = m.fb_position.load();
                    std::cout << "✅ Zero set: current position is now 0 for all motors.\n";
                } else {
                    break; // Enter pressed — move on to control
                }
            }
        }

        probing = false;
        probe.join();
    }

    if (!running) {
        reader.join();
        close(s);
        return 0;
    }

    // Now start sending control frames
    // target_position=0 means "hold at the zeroed position" (writer adds zero_offset)
    for (auto& m : motors)
        m.target_position = 0.0;
    std::thread writer(writer_thread, s, std::ref(motors));

    // CLI loop — prints feedback then blocks waiting for your input
    while (running) {
        std::cout << "\n--- Motor Feedback ---\n";
        for (size_t i = 0; i < motors.size(); i++) {
            std::cout << "[" << i << "] ID:" << motors[i].motor_id
                      << "  pos=" << (motors[i].fb_position.load() - motors[i].zero_offset.load()) << " rad"
                      << "  vel=" << motors[i].fb_velocity.load() << " rad/s"
                      << "  tor=" << motors[i].fb_torque.load()   << " Nm"
                      << "  kp="  << motors[i].kp.load()
                      << "  kd="  << motors[i].kd.load() << "\n";
        }
        std::cout << "cmd> " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) { running = false; break; }

        if (line == "q") {
            running = false;
        } else if (line.size() > 4 && line.substr(0, 4) == "all ") {
            double deg = std::stod(line.substr(4));
            for (auto& m : motors) m.target_position = deg * M_PI / 180.0;
            std::cout << "All motors -> " << deg << " deg\n";
        } else if (line.size() > 7 && line.substr(0, 7) == "all kp ") {
            double val = std::stod(line.substr(7));
            for (auto& m : motors) m.kp = val;
            std::cout << "All motors kp -> " << val << "\n";
        } else if (line.size() > 7 && line.substr(0, 7) == "all kd ") {
            double val = std::stod(line.substr(7));
            for (auto& m : motors) m.kd = val;
            std::cout << "All motors kd -> " << val << "\n";
        } else {
            std::istringstream ss(line);
            std::string token; double val;
            if (ss >> token >> val) {
                // Check for "kp" or "kd" prefix: e.g. "kp 10 5.0" or "kd 10 1.5"
                bool is_kp = (token == "kp");
                bool is_kd = (token == "kd");
                if (is_kp || is_kd) {
                    int id2; double val2;
                    // token already read as "kp"/"kd", val is motor id, val2 is the gain
                    id2 = static_cast<int>(val);
                    if (ss >> val2) {
                        MotorControl* target = nullptr;
                        for (auto& m : motors) if (m.motor_id == id2) { target = &m; break; }
                        if (!target && id2 >= 0 && static_cast<size_t>(id2) < motors.size()) target = &motors[id2];
                        if (target) {
                            if (is_kp) { target->kp = val2; std::cout << "Motor ID:" << target->motor_id << " kp -> " << val2 << "\n"; }
                            else       { target->kd = val2; std::cout << "Motor ID:" << target->motor_id << " kd -> " << val2 << "\n"; }
                        } else {
                            std::cout << "Unknown motor ID or index: " << id2 << "\n";
                        }
                    } else {
                        std::cout << "Usage: kp <motor_id> <value>  |  kd <motor_id> <value>\n";
                    }
                } else {
                    // Positional command: token is motor id, val is degrees
                    int id = std::stoi(token);
                    MotorControl* target = nullptr;
                    for (auto& m : motors) if (m.motor_id == id) { target = &m; break; }
                    if (!target && id >= 0 && static_cast<size_t>(id) < motors.size()) target = &motors[id];
                    if (target) {
                        target->target_position = val * M_PI / 180.0;
                        std::cout << "Motor ID:" << target->motor_id << " -> " << val << " deg\n";
                    } else {
                        std::cout << "Unknown motor ID or index: " << id << "\n";
                        std::cout << "Usage: <motor_id> <deg>  |  kp <motor_id> <val>  |  kd <motor_id> <val>  |  all <deg>  |  q\n";
                    }
                }
            } else {
                std::cout << "Usage: <motor_id> <deg>  |  kp <motor_id> <val>  |  kd <motor_id> <val>  |  all <deg>  |  all kp <val>  |  all kd <val>  |  q\n";
            }
        }
    }

    // Cleanup
    running = false;
    reader.join();
    writer.join();
    for (auto& m : motors) {
        uint32_t ext_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | m.motor_id;
        send_frame(s, ext_id, nullptr, 0);
    }
    close(s);
    std::cout << "Shutdown complete.\n";
    return 0;
}