/*
 * Sinusoidal Actuator Model Test — Inspired by Rhoban BAM
 * 
 * This test executes sinusoidal trajectories to characterize motor behavior
 * under various frequencies and amplitudes. Data is logged to CSV for analysis.
 *
 * Features:
 *   - Multiple sinusoidal trajectory types (sin_time_square, sin_sin, simple_sin)
 *   - Configurable frequency, amplitude, and control gains
 *   - Real-time data logging (time, target, actual, velocity, torque)
 *   - CSV export for post-processing
 *
 * Build:
 *   g++ -std=c++17 -I../include -o sinusoidal_actuator_test sinusoidal_actuator_test.cpp \
 *       ../src/can_interfaces.cpp -pthread
 *
 * Run:
 *   sudo ./sinusoidal_actuator_test <motor_id> <trajectory_type> [motor_type] [kp] [kd] [frequency] [amplitude]
 *   
 *   Example:
 *     sudo ./sinusoidal_actuator_test 11 sin_time_square rs03 10 2 2.0 45
 *     sudo ./sinusoidal_actuator_test 11 sin_sin 8 1.5 1.5 30
 */

#include "../include/motor_control.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <cmath>
#include <cstdlib>
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>

void enable_realtime() {
    mlockall(MCL_CURRENT | MCL_FUTURE);

    struct sched_param param;
    param.sched_priority = 90;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        perror("sched_setscheduler");
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
        perror("sched_setaffinity");
    }
}

inline void enable_motor(CanInterface &can, int motor_id)
{
    uint32_t ext_id = (CommType::ENABLE << 24) | (0 << 16) | (HOST_ID << 8) | motor_id;
    uint8_t data[1] = {0};
    can.write_frame(ext_id, data, 0);
}

// ─────────────────────────────────────────────────────────────
// TRAJECTORY GENERATORS
// ─────────────────────────────────────────────────────────────

class SinusoidalTrajectory {
public:
    enum Type {
        SIN_TIME_SQUARE,  // sin(t²) — increasing frequency
        SIN_SIN,          // sin(ωt) + 0.5*sin(2ωt) — multi-frequency
        SIMPLE_SIN        // sin(ωt) — basic sinusoid
    };

    Type type;
    double frequency_hz;    // fundamental frequency in Hz
    double amplitude_deg;   // amplitude in degrees
    double t_start;         // start time

    SinusoidalTrajectory(Type t, double freq, double amp)
        : type(t), frequency_hz(freq), amplitude_deg(amp), t_start(0.0) {}

    /**
     * Generate target position (degrees) for a given time
     * @param t_elapsed: elapsed time in seconds since start
     * @return target position in degrees
     */
    double get_target_position(double t_elapsed) const {
        const double omega = 2.0 * M_PI * frequency_hz;
        double target = 0.0;

        switch (type) {
            case SIN_TIME_SQUARE:
                // Progressive frequency increase: sin(t²)
                // Useful for sweeping through frequencies smoothly
                target = amplitude_deg * std::sin(omega * t_elapsed * t_elapsed);
                break;

            case SIN_SIN:
                // Superposition of two sine waves
                // Tests actuator response at multiple frequencies simultaneously
                target = amplitude_deg * (std::sin(omega * t_elapsed) +
                                         0.5 * std::sin(2.0 * omega * t_elapsed));
                break;

            case SIMPLE_SIN:
            default:
                // Simple sinusoid: easiest case
                target = amplitude_deg * std::sin(omega * t_elapsed);
                break;
        }

        return target;
    }

    /**
     * Get velocity target (numerical derivative)
     */
    double get_target_velocity(double t_elapsed, double dt = 0.001) const {
        double pos_plus  = get_target_position(t_elapsed + dt);
        double pos_minus = get_target_position(t_elapsed - dt);
        return (pos_plus - pos_minus) / (2.0 * dt);
    }

    static const char* type_name(Type t) {
        switch (t) {
            case SIN_TIME_SQUARE: return "sin_time_square";
            case SIN_SIN:         return "sin_sin";
            case SIMPLE_SIN:      return "simple_sin";
            default:              return "unknown";
        }
    }
};

// ─────────────────────────────────────────────────────────────
// DATA LOGGING
// ─────────────────────────────────────────────────────────────

struct LogEntry {
    double time_s;
    double target_pos_deg;
    double target_vel_deg_s;
    double actual_pos_deg;
    double actual_vel_deg_s;
    double actual_torque_nm;
    double kp;
    double kd;
    double error_deg;
    double power_w;  // v * i estimation

    // CSV header
    static std::string csv_header() {
        return "time_s,target_pos_deg,target_vel_deg_s,actual_pos_deg,"
               "actual_vel_deg_s,actual_torque_nm,kp,kd,error_deg,power_w";
    }

    std::string to_csv() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << time_s << ","
            << target_pos_deg << ","
            << target_vel_deg_s << ","
            << actual_pos_deg << ","
            << actual_vel_deg_s << ","
            << actual_torque_nm << ","
            << kp << ","
            << kd << ","
            << error_deg << ","
            << power_w;
        return oss.str();
    }
};

class DataLogger {
private:
    std::ofstream file_;
    std::vector<LogEntry> entries_;
    std::string filename_;

public:
    DataLogger(const std::string& filename) : filename_(filename) {
        file_.open(filename, std::ios::out);
        if (!file_.is_open()) {
            std::cerr << "❌ Failed to open log file: " << filename << std::endl;
            return;
        }
        file_ << LogEntry::csv_header() << "\n";
        file_.flush();
    }

    ~DataLogger() {
        if (file_.is_open()) file_.close();
    }

    void log(const LogEntry& entry) {
        entries_.push_back(entry);
        file_ << entry.to_csv() << "\n";
        file_.flush();
    }

    void close() {
        if (file_.is_open()) file_.close();
        std::cout << "✅ Log saved: " << filename_ << " (" << entries_.size() << " entries)\n";
    }

    // Statistics
    void print_statistics() const {
        if (entries_.empty()) return;

        std::vector<double> errors;
        double min_torque = entries_[0].actual_torque_nm;
        double max_torque = entries_[0].actual_torque_nm;
        double total_power = 0.0;

        for (const auto& e : entries_) {
            errors.push_back(std::abs(e.error_deg));
            min_torque = std::min(min_torque, e.actual_torque_nm);
            max_torque = std::max(max_torque, e.actual_torque_nm);
            total_power += e.power_w;
        }

        double mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        double max_error = *std::max_element(errors.begin(), errors.end());

        std::cout << "\n─── TEST STATISTICS ───\n";
        std::cout << "Mean Absolute Error: " << std::fixed << std::setprecision(4) 
                  << mean_error << " deg\n";
        std::cout << "Max Error:           " << max_error << " deg\n";
        std::cout << "Torque Range:        [" << min_torque << ", " << max_torque << "] Nm\n";
        std::cout << "Total Energy Used:   " << (total_power / entries_.size()) * entries_.size() * 0.01 
                  << " (relative)\n";
        std::cout << "Samples Collected:   " << entries_.size() << "\n";
    }
};

// ─────────────────────────────────────────────────────────────
// TEST EXECUTOR
// ─────────────────────────────────────────────────────────────

class SinusoidalTestRunner {
private:
    CanInterface& can_;
    std::vector<Motor>& motors_;
    int motor_id_;
    double kp_;
    double kd_;
    double torque_limit_nm_;
    double dt_s_;  // control loop timestep

public:
    SinusoidalTestRunner(CanInterface& can, std::vector<Motor>& motors,
                         int motor_id, double kp, double kd, double torque_limit = 20.0)
        : can_(can), motors_(motors), motor_id_(motor_id),
          kp_(kp), kd_(kd), torque_limit_nm_(torque_limit), dt_s_(0.005) {}

    /**
     * Execute sinusoidal trajectory test
     * @param traj: trajectory generator
     * @param duration_s: total test duration
     * @return log filename
     */
    std::string run_test(const SinusoidalTrajectory& traj, double duration_s) {
        // Find motor
        auto motor_it = std::find_if(motors_.begin(), motors_.end(),
                                     [this](const Motor& m) { return m.id == motor_id_; });
        if (motor_it == motors_.end()) {
            std::cerr << "❌ Motor " << motor_id_ << " not found\n";
            return "";
        }
        Motor& motor = *motor_it;

        // Create filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << "sinusoidal_test_" << motor_id_ << "_"
            << SinusoidalTrajectory::type_name(traj.type) << "_"
            << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
        std::string logfile = oss.str();

        DataLogger logger(logfile);

        std::cout << "\n━━━ SINUSOIDAL ACTUATOR TEST ━━━\n";
        std::cout << "Motor ID:       " << motor_id_ << "\n";
        std::cout << "Trajectory:     " << SinusoidalTrajectory::type_name(traj.type) << "\n";
        std::cout << "Frequency:      " << traj.frequency_hz << " Hz\n";
        std::cout << "Amplitude:      " << traj.amplitude_deg << " deg\n";
        std::cout << "Duration:       " << duration_s << " s\n";
        std::cout << "KP/KD:          " << kp_ << " / " << kd_ << "\n";
        std::cout << "Torque Limit:   " << torque_limit_nm_ << " Nm\n";
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";

        auto start_time = std::chrono::high_resolution_clock::now();
        int step_count = 0;

        // Home motor first
        std::cout << "⏳ Homing motor...\n";
        send_position(can_, motor_id_, 0.0, 5.0, 0.5, motor.type, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        collect_status(can_, motors_);

        std::cout << "🎬 Starting trajectory...\n";

        // Main control loop
        while (true) {
            auto loop_start = std::chrono::high_resolution_clock::now();
            auto elapsed_duration = loop_start - start_time;
            double t_elapsed = std::chrono::duration<double>(elapsed_duration).count();

            if (t_elapsed > duration_s) break;

            // Get trajectory targets
            double target_pos = traj.get_target_position(t_elapsed);
            double target_vel = traj.get_target_velocity(t_elapsed);

            // Simple PD control with feedforward
            double actual_pos_deg = motor.pos * 180.0 / M_PI;
            double error = target_pos - actual_pos_deg;
            double torque_ff = 0.0;  // Could add gravity compensation here
            
            // Clamp torque
            if (std::abs(torque_ff) > torque_limit_nm_)
                torque_ff = (torque_ff > 0) ? torque_limit_nm_ : -torque_limit_nm_;

            // Send control command
            send_position(can_, motor_id_, target_pos * M_PI / 180.0,
                         kp_, kd_, motor.type, torque_ff);

            // Read motor status
            collect_status(can_, motors_);

            // Estimate power (simplified)
            double power = motor.torque * motor.filtered_vel;

            // Log entry
            LogEntry entry;
            entry.time_s = t_elapsed;
            entry.target_pos_deg = target_pos;
            entry.target_vel_deg_s = target_vel;
            entry.actual_pos_deg = motor.pos * 180.0 / M_PI;
            entry.actual_vel_deg_s = motor.filtered_vel * 180.0 / M_PI;
            entry.actual_torque_nm = motor.torque;
            entry.kp = kp_;
            entry.kd = kd_;
            entry.error_deg = error;
            entry.power_w = power;

            logger.log(entry);

            // Progress indicator
            if (step_count % 50 == 0) {
                std::cout << "  [" << std::fixed << std::setprecision(1) << t_elapsed << "s / " 
                          << duration_s << "s] error=" << std::setprecision(3) << error 
                          << "° torque=" << motor.torque << "Nm\n";
            }

            // Sleep to maintain timestep
            auto loop_duration = std::chrono::high_resolution_clock::now() - loop_start;
            auto sleep_time = std::chrono::milliseconds(
                static_cast<int>(dt_s_ * 1000)) - loop_duration;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }

            step_count++;
        }

        std::cout << "✅ Test completed\n";
        logger.print_statistics();
        logger.close();

        return logfile;
    }
};

// ─────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <motor_id> <trajectory_type> [motor_type] [kp] [kd] [frequency_hz] [amplitude_deg]\n\n"
                  << "Trajectory types:\n"
                  << "  sin_time_square  — sin(t²) progressive frequency\n"
                  << "  sin_sin          — sin(ωt) + 0.5*sin(2ωt)\n"
                  << "  simple_sin       — sin(ωt) basic sinusoid\n\n"
                  << "Examples:\n"
                  << "  sudo ./sinusoidal_actuator_test 11 simple_sin rs03 10 2 2.0 45\n"
                  << "  sudo ./sinusoidal_actuator_test 11 simple_sin 10 2 2.0 45  # uses default rs03\n";
        return 1;
    }

    int motor_id = std::atoi(argv[1]);
    std::string traj_name = argv[2];
    std::string motor_type = "rs03";
    int arg_index = 3;

    auto is_number = [](const std::string& s) {
        if (s.empty()) return false;
        char* endptr = nullptr;
        std::strtod(s.c_str(), &endptr);
        return endptr != s.c_str() && *endptr == '\0';
    };

    if (argc > 3 && !is_number(argv[3])) {
        motor_type = argv[3];
        arg_index = 4;
    }

    double kp = (argc > arg_index) ? std::atof(argv[arg_index]) : 10.0;
    double kd = (argc > arg_index + 1) ? std::atof(argv[arg_index + 1]) : 2.0;
    double frequency = (argc > arg_index + 2) ? std::atof(argv[arg_index + 2]) : 1.0;
    double amplitude = (argc > arg_index + 3) ? std::atof(argv[arg_index + 3]) : 30.0;
    enable_realtime();
    // Determine trajectory type
    SinusoidalTrajectory::Type traj_type = SinusoidalTrajectory::SIMPLE_SIN;
    if (traj_name == "sin_time_square")
        traj_type = SinusoidalTrajectory::SIN_TIME_SQUARE;
    else if (traj_name == "sin_sin")
        traj_type = SinusoidalTrajectory::SIN_SIN;

    // Initialize CAN
    CanInterface can;
    if (!can.init("can0")) {
        std::cerr << "❌ Failed to initialize CAN interface\n";
        return 1;
    }
    std::cout << "✅ CAN interface initialized\n";

    // Create motor and enable it
    Motor motor;
    motor.id = motor_id;
    motor.type = motor_type;  // Use passed motor type or default rs03

    std::vector<Motor> motors = {motor};

    // Enable motor
    std::cout << "⚡ Enabling motor " << motor_id << "...\n";
    enable_motor(can, motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Set MIT mode
    send_mode(can, motor_id, ControlMode::MIT_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Create test runner and execute
    SinusoidalTestRunner test_runner(can, motors, motor_id, kp, kd);
    SinusoidalTrajectory trajectory(traj_type, frequency, amplitude);
    
    std::string logfile = test_runner.run_test(trajectory, 10.0);  // 10 second test

    std::cout << "\n💾 Data saved to: " << logfile << "\n";
    std::cout << "📊 Use 'python3 plot_sinusoidal_test.py " << logfile 
              << "' to visualize\n";

    return 0;
}
