/*
 * hello_motor.cpp — Build this file yourself, step by step.
 *
 * GOAL: Send one position command to one motor and print what it reports back.
 *
 * Build when ready:
 *   g++ -std=c++17 -I../include -o hello_motor hello_motor.cpp
 *
 * Run:
 *   sudo ./hello_motor <motor_id> <degrees>
 *   e.g.  sudo ./hello_motor 11 45
 *
 * ============================================================
 *  WHAT YOU NEED TO KNOW BEFORE WRITING ANY CODE
 * ============================================================
 *
 *  1. CAN ID LAYOUT (29-bit extended)
 *     Each message has a 29-bit ID that carries meaning:
 *
 *       bits [28:24]  comm_type   — what command? (5 bits)
 *       bits [23:16]  extra       — depends on command
 *       bits [15:8]   sender_id   — who is sending  (PC = 0xFF)
 *       bits [7:0]    motor_id    — who should act
 *
 *     Example — ENABLE (type=3) to motor 11:
 *       ext_id = (3 << 24) | (0 << 16) | (0xFF << 8) | 11
 *
 *  2. COMM TYPES (from protocol_completed.h → CommType namespace)
 *     ENABLE           = 3    no data, wakes motor
 *     DISABLE          = 4    no data, idles motor
 *     WRITE_PARAMETER  = 18   8 bytes: [param_id LE 2B][padding 2B][value 4B]
 *     OPERATION_CONTROL= 1    8 bytes: position/vel/kp/kd packed as uint16 BE
 *                             torque_ff goes in CAN ID bits [23:8]
 *     OPERATION_STATUS = 2    reply FROM motor: pos/vel/torque/temp/fault
 *
 *  3. PARAMETER IDS (from protocol_completed.h → ParamID namespace)
 *     ParamID::MODE     = 0x7005   int8:  0=MIT, 1=Position, 2=Speed, 3=Torque
 *     ParamID::VELOCITY_LIMIT = 0x7017  float: rad/s
 *     ParamID::TORQUE_LIMIT   = 0x700B  float: Nm
 *
 *  4. SCALE TABLES (from protocol_completed.h → ModelScale::RS03)
 *     POSITION = 4π  rad      signed ±
 *     VELOCITY = 50  rad/s    signed ±
 *     TORQUE   = 60  Nm       signed ±
 *     KP       = 5000 Nm/rad  unsigned 0–max
 *     KD       = 100  Nm·s/rad unsigned 0–max
 *
 *  5. ENCODING FORMULAS
 *     Signed value  (pos, vel, torque):
 *       raw_u16 = (value / max + 1.0) * 32767      range → [0, 65534]
 *       0 maps to 32767 (midpoint)
 *
 *     Unsigned value (kp, kd):
 *       raw_u16 = (value / max) * 65535             range → [0, 65535]
 *
 *     DECODING (what motor replies):
 *       value = (raw_u16 / 32767.0 - 1.0) * max
 *
 *  6. BYTE ORDER
 *     Control frame data bytes   → BIG-endian    (use pack_u16_be)
 *     WRITE_PARAMETER param_id   → LITTLE-endian (use pack_u16_le)
 *     WRITE_PARAMETER float val  → LITTLE-endian (use pack_float_le / memcpy)
 *
 *  7. USEFUL API FROM can_interfaces.h
 *     CanInterface can;
 *     can.init("can0");                            // open the socket
 *     can.write_frame(ext_id, data, dlc);          // send a frame
 *     can.read_frame(&frame, timeout_ms);          // receive a frame (returns bool)
 *     can.is_ready();                              // check if socket open
 *
 *     The received frame is a Linux struct can_frame:
 *       frame.can_id   — full 29-bit ID with CAN_EFF_FLAG set
 *       frame.can_dlc  — number of data bytes
 *       frame.data[]   — up to 8 payload bytes
 *
 *     To extract fields from a received can_id:
 *       uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
 *       int      src_id    = (frame.can_id >>  8) & 0xFF;
 *
 * ============================================================
 *  YOUR TASKS — fill in each section below
 * ============================================================
 */

#include "../include/motor_control.hpp"
#include <cctype>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <set>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <memory>
#include <iomanip>
#include <ctime>
#include <sched.h>
#include <sys/mman.h>
#include <pthread.h>
// torque — motor can be rotated freely by hand.

// CLI: <cmd> <target_deg> <id> <type> [<id> <type> ...]
//
//  z               — zero ALL listed motors
//  zp <motor_id>   — zero ONE motor (put its id as target_deg, list all motors)
//  r               — passive read all (zero torque)
//  w               — move+read all to target_deg
//  t <torque>     — apply torque to all motors and read status
//  s               — move through multiple target angles and hold each one
struct SequenceStep
{
    double angle_deg = 0.0;
    double kp = 1.0;
    double kd = 0.5;
    double torque_ff = 0.0;
    double velocity_deg_s = 0.0;
};

struct PlotSample
{
    double time_s = 0.0;
    double target_deg = 0.0;
    double actual_deg = 0.0;
    double observed_hz = 0.0;
    double target_vel_deg_s = 0.0;
    double actual_vel_deg_s = 0.0;
    double actual_torque = 0.0;
};

// ---- Logging infrastructure for non-blocking I/O ----------------------------
struct LogEntry
{
    long timestamp_ms = 0;
    int motor_id = 0;
    int pos_raw = 0;
    double target_deg = 0.0;
    double target_vel_deg_s = 0.0;
    double actual_deg = 0.0;
    double error_deg = 0.0;
    double vel_deg_s = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double torque_ff = 0.0;
    double actual_torque = 0.0;
    double loop_dt_us = 0.0;
    double observed_hz = 0.0;
};

// ---- Base class for plotter strategies -----------------------------------
class PlotterInterface
{
public:
    virtual ~PlotterInterface() = default;
    virtual bool ready() const = 0;
    virtual void add_sample(const LogEntry &entry) = 0;
    virtual void redraw_all() = 0;
};

// Simple single-window plotter for torque vs time
class RealtimeTorquePlotter : public PlotterInterface
{
public:
    explicit RealtimeTorquePlotter(const std::vector<Motor> &motors)
    {
        if (std::system("command -v gnuplot >/dev/null 2>&1") != 0)
        {
            std::cerr << "Warning: gnuplot is not installed. CSV logging will continue.\n";
            return;
        }

        for (const auto &m : motors)
            samples_[m.id] = {};

        pipe_torque_ = popen("gnuplot -persist", "w");

        if (pipe_torque_)
        {
            std::fprintf(pipe_torque_, "set term qt size 1200,600 title 'Torque vs Time'\n");
            std::fprintf(pipe_torque_, "set title 'Multi-Sine Torque Excitation'\n");
            std::fprintf(pipe_torque_, "set xlabel 'Time (s)'\n");
            std::fprintf(pipe_torque_, "set ylabel 'Torque (Nm)'\n");
            std::fprintf(pipe_torque_, "set grid\n");
            std::fprintf(pipe_torque_, "set key outside bottom center horizontal\n");
            std::fflush(pipe_torque_);
        }
    }

    ~RealtimeTorquePlotter()
    {
        if (pipe_torque_) pclose(pipe_torque_);
    }

    bool ready() const override
    {
        return pipe_torque_ != nullptr;
    }

    void add_sample(const LogEntry &entry) override
    {
        TorqueSample s;
        s.time_s = entry.timestamp_ms / 1000.0;
        s.commanded = entry.torque_ff;
        s.actual = entry.actual_torque;
        samples_[entry.motor_id].push_back(s);
    }

    void redraw_all() override
    {
        if (samples_.empty() || !pipe_torque_)
            return;

        // Write data files
        for (const auto &entry : samples_)
        {
            std::ostringstream path;
            path << "/tmp/robstride_torque_" << entry.first << ".dat";
            std::ofstream ofs(path.str(), std::ios::trunc);
            if (!ofs)
                continue;

            // Columns: 1=time  2=actual_torque  3=commanded_torque
            for (const auto &sample : entry.second)
                ofs << sample.time_s << " " << sample.actual << " " << sample.commanded << "\n";
        }

        // Redraw
        std::fprintf(pipe_torque_, "reset\n");
        std::fprintf(pipe_torque_, "set term qt size 1200,600 title 'Torque vs Time'\n");
        std::fprintf(pipe_torque_, "set title 'Multi-Sine Torque Excitation'\n");
        std::fprintf(pipe_torque_, "set xlabel 'Time (s)'\n");
        std::fprintf(pipe_torque_, "set ylabel 'Torque (Nm)'\n");
        std::fprintf(pipe_torque_, "set grid\n");
        std::fprintf(pipe_torque_, "set key outside bottom center horizontal\n");
        bool first = true;
        std::fprintf(pipe_torque_, "plot ");
        for (const auto &entry : samples_)
        {
            std::ostringstream path;
            path << "/tmp/robstride_torque_" << entry.first << ".dat";
            if (!first) std::fprintf(pipe_torque_, ", ");
            std::fprintf(pipe_torque_, "'%s' using 1:3 with lines lw 1 dt 2 lc rgb '#888888' title 'Cmd M%d'", path.str().c_str(), entry.first);
            std::fprintf(pipe_torque_, ", '%s' using 1:2 with lines lw 2 title 'Actual M%d'", path.str().c_str(), entry.first);
            first = false;
        }
        std::fprintf(pipe_torque_, "\n");
        std::fflush(pipe_torque_);
    }

private:
    FILE *pipe_torque_ = nullptr;
    struct TorqueSample { double time_s; double commanded; double actual; };
    std::map<int, std::vector<TorqueSample>> samples_;
};

class RealtimeGnuplot : public PlotterInterface
{
public:
    explicit RealtimeGnuplot(const std::vector<Motor> &motors)
    {
        if (std::system("command -v gnuplot >/dev/null 2>&1") != 0)
        {
            std::cerr << "Warning: gnuplot is not installed. CSV logging will continue.\n";
            return;
        }

        for (const auto &m : motors)
            samples_[m.id] = {};

        // Open four separate gnuplot windows — one per plot type
        pipe_angle_ = popen("gnuplot -persist", "w");
        pipe_freq_  = popen("gnuplot -persist", "w");
        pipe_vel_   = popen("gnuplot -persist", "w");
        pipe_accel_ = popen("gnuplot -persist", "w");
        pipe_inertia_ = popen("gnuplot -persist", "w");

        if (pipe_angle_)
        {
            std::fprintf(pipe_angle_, "set term qt size 900,400 title 'Position vs Time'\n");
            std::fprintf(pipe_angle_, "set title 'Angle sequence tracking'\n");
            std::fprintf(pipe_angle_, "set xlabel 'Time (s)'\n");
            std::fprintf(pipe_angle_, "set ylabel 'Angle (deg)'\n");
            std::fprintf(pipe_angle_, "set grid\n");
            std::fprintf(pipe_angle_, "set key outside bottom center horizontal\n");
            std::fflush(pipe_angle_);
        }
        if (pipe_freq_)
        {
            std::fprintf(pipe_freq_, "set term qt size 900,400 title 'Observed Loop Frequency'\n");
            std::fprintf(pipe_freq_, "set title 'Observed Loop Frequency'\n");
            std::fprintf(pipe_freq_, "set xlabel 'Time (s)'\n");
            std::fprintf(pipe_freq_, "set ylabel 'Frequency (Hz)'\n");
            std::fprintf(pipe_freq_, "set grid\n");
            std::fprintf(pipe_freq_, "set key outside bottom center horizontal\n");
            std::fflush(pipe_freq_);
        }
        if (pipe_vel_)
        {
            std::fprintf(pipe_vel_, "set term qt size 900,400 title 'Velocity vs Time'\n");
            std::fprintf(pipe_vel_, "set title 'Velocity over time'\n");
            std::fprintf(pipe_vel_, "set xlabel 'Time (s)'\n");
            std::fprintf(pipe_vel_, "set ylabel 'Velocity (deg/s)'\n");
            std::fprintf(pipe_vel_, "set grid\n");
            std::fprintf(pipe_vel_, "set key outside bottom center horizontal\n");
            std::fflush(pipe_vel_);
        }
        if (pipe_accel_)
        {
            std::fprintf(pipe_accel_, "set term qt size 900,400 title 'Angular Acceleration vs Time'\n");
            std::fprintf(pipe_accel_, "set title 'Angular Acceleration (smoothed)'\n");
            std::fprintf(pipe_accel_, "set xlabel 'Time (s)'\n");
            std::fprintf(pipe_accel_, "set ylabel 'Acceleration (deg/s^2)'\n");
            std::fprintf(pipe_accel_, "set grid\n");
            std::fprintf(pipe_accel_, "set key outside bottom center horizontal\n");
            std::fflush(pipe_accel_);
        }
        if (pipe_inertia_)
        {
            std::fprintf(pipe_inertia_, "set term qt size 900,400 title 'Estimated Inertia vs Time'\n");
            std::fprintf(pipe_inertia_, "set title 'Estimated Moment of Inertia (I = \u03c4 / \u03b1)'\n");
            std::fprintf(pipe_inertia_, "set xlabel 'Time (s)'\n");
            std::fprintf(pipe_inertia_, "set ylabel 'Inertia (Nm/(rad/s^2))'\n");
            std::fprintf(pipe_inertia_, "set grid\n");
            std::fprintf(pipe_inertia_, "set key outside bottom center horizontal\n");
            std::fflush(pipe_inertia_);
        }
    }

    ~RealtimeGnuplot()
    {
        if (pipe_angle_) pclose(pipe_angle_);
        if (pipe_freq_)  pclose(pipe_freq_);
        if (pipe_vel_)   pclose(pipe_vel_);
        if (pipe_accel_) pclose(pipe_accel_);
        if (pipe_inertia_) pclose(pipe_inertia_);
    }

    bool ready() const override
    {
        return pipe_angle_ != nullptr;
    }

    void add_sample(const LogEntry &entry) override
    {
        samples_[entry.motor_id].push_back({entry.timestamp_ms / 1000.0, entry.target_deg, entry.actual_deg, entry.observed_hz, entry.target_vel_deg_s, entry.vel_deg_s, entry.actual_torque});
    }

private:
    void write_data_files() const
    {
        static const int SMOOTH_WINDOW = 100; // 100-sample (100ms) moving average for clean acceleration

        for (const auto &entry : samples_)
        {
            std::ostringstream path;
            path << "/tmp/robstride_plot_" << entry.first << ".dat";
            std::ofstream ofs(path.str(), std::ios::trunc);
            if (!ofs)
                continue;

            const auto &vec = entry.second;

            // First pass: compute raw acceleration for every sample
            std::vector<double> raw_accel(vec.size(), 0.0);
            for (size_t i = 1; i < vec.size(); i++)
            {
                double dt = vec[i].time_s - vec[i - 1].time_s;
                if (dt > 1e-9)
                    raw_accel[i] = (vec[i].actual_vel_deg_s - vec[i - 1].actual_vel_deg_s) / dt;
            }

            // Second pass: moving average smoothing + inertia
            double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
            int n_regression = 0;

            for (size_t i = 0; i < vec.size(); i++)
            {
                const auto &sample = vec[i];

                // Smoothed acceleration (centred window, clamped at edges)
                double sum = 0.0;
                int count = 0;
                int half = SMOOTH_WINDOW / 2;
                for (int j = -(half); j <= half; j++)
                {
                    int idx = (int)i + j;
                    if (idx >= 0 && idx < (int)vec.size())
                    {
                        sum += raw_accel[idx];
                        count++;
                    }
                }
                double smoothed_accel = (count > 0) ? sum / count : 0.0;

                // Convert to rad/s^2 for inertia calculation
                double accel_rad_s2 = smoothed_accel * M_PI / 180.0;

                // Inertia: I = τ / α  (Nm / (rad/s^2) = kg·m²)
                // Only meaningful when acceleration is significant
                double inertia = 0.0;
                if (std::fabs(accel_rad_s2) > 0.5)  // threshold to avoid division by tiny values
                {
                    inertia = sample.actual_torque / accel_rad_s2;
                    // Cap inertia to prevent massive spikes on the plot when accel is near zero
                    if (inertia > 5.0) inertia = 5.0;
                    if (inertia < -5.0) inertia = -5.0;
                }

                // Columns: 1=time 2=actual_deg 3=target_deg 4=freq 5=vel_actual 6=vel_target 7=smoothed_accel 8=inertia
                ofs << sample.time_s << " "
                    << sample.actual_deg << " "
                    << sample.target_deg << " "
                    << sample.observed_hz << " "
                    << sample.actual_vel_deg_s << " "
                    << sample.target_vel_deg_s << " "
                    << smoothed_accel << " "
                    << inertia << "\n";

                // Accumulate data for true inertia estimation (linear regression slope)
                if (std::fabs(accel_rad_s2) > 2.0)
                {
                    sum_x += accel_rad_s2;
                    sum_y += sample.actual_torque;
                    sum_xx += accel_rad_s2 * accel_rad_s2;
                    sum_xy += accel_rad_s2 * sample.actual_torque;
                    n_regression++;
                }
            }

            if (n_regression > 1)
            {
                double denominator = (n_regression * sum_xx - sum_x * sum_x);
                if (std::fabs(denominator) > 1e-9)
                    estimated_inertias_[entry.first] = (n_regression * sum_xy - sum_x * sum_y) / denominator;
            }
        }
    }

public:
    void redraw_all() override
    {
        static const int SMOOTH_WINDOW = 100; // 100-sample (100ms) moving average for clean acceleration

        for (const auto &entry : samples_)
        {
            std::ostringstream path;
            path << "/tmp/robstride_plot_" << entry.first << ".dat";
            std::ofstream ofs(path.str(), std::ios::trunc);
            if (!ofs)
                continue;

            const auto &vec = entry.second;

            // First pass: compute raw acceleration for every sample
            std::vector<double> raw_accel(vec.size(), 0.0);
            for (size_t i = 1; i < vec.size(); i++)
            {
                double dt = vec[i].time_s - vec[i - 1].time_s;
                if (dt > 1e-9)
                    raw_accel[i] = (vec[i].actual_vel_deg_s - vec[i - 1].actual_vel_deg_s) / dt;
            }

            // Second pass: moving average smoothing + inertia
            double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
            int n_regression = 0;

            for (size_t i = 0; i < vec.size(); i++)
            {
                const auto &sample = vec[i];

                // Smoothed acceleration (centred window, clamped at edges)
                double sum = 0.0;
                int count = 0;
                int half = SMOOTH_WINDOW / 2;
                for (int j = -(half); j <= half; j++)
                {
                    int idx = (int)i + j;
                    if (idx >= 0 && idx < (int)vec.size())
                    {
                        sum += raw_accel[idx];
                        count++;
                    }
                }
                double smoothed_accel = (count > 0) ? sum / count : 0.0;

                // Convert to rad/s^2 for inertia calculation
                double accel_rad_s2 = smoothed_accel * M_PI / 180.0;

                // Inertia: I = τ / α  (Nm / (rad/s^2) = kg·m²)
                // Only meaningful when acceleration is significant
                double inertia = 0.0;
                if (std::fabs(accel_rad_s2) > 0.5)  // threshold to avoid division by tiny values
                {
                    inertia = sample.actual_torque / accel_rad_s2;
                    // Cap inertia to prevent massive spikes on the plot when accel is near zero
                    if (inertia > 5.0) inertia = 5.0;
                    if (inertia < -5.0) inertia = -5.0;
                }

                // Columns: 1=time 2=actual_deg 3=target_deg 4=freq 5=vel_actual 6=vel_target 7=smoothed_accel 8=inertia
                ofs << sample.time_s << " "
                    << sample.actual_deg << " "
                    << sample.target_deg << " "
                    << sample.observed_hz << " "
                    << sample.actual_vel_deg_s << " "
                    << sample.target_vel_deg_s << " "
                    << smoothed_accel << " "
                    << inertia << "\n";

                // Accumulate data for true inertia estimation (linear regression slope)
                if (std::fabs(accel_rad_s2) > 2.0)
                {
                    sum_x += accel_rad_s2;
                    sum_y += sample.actual_torque;
                    sum_xx += accel_rad_s2 * accel_rad_s2;
                    sum_xy += accel_rad_s2 * sample.actual_torque;
                    n_regression++;
                }
            }

            if (n_regression > 1)
            {
                double denominator = (n_regression * sum_xx - sum_x * sum_x);
                if (std::fabs(denominator) > 1e-9)
                    estimated_inertias_[entry.first] = (n_regression * sum_xy - sum_x * sum_y) / denominator;
            }
        }
    }

private:
    FILE *pipe_angle_ = nullptr;
    FILE *pipe_freq_  = nullptr;
    FILE *pipe_vel_   = nullptr;
    FILE *pipe_accel_ = nullptr;
    FILE *pipe_inertia_ = nullptr;
    std::map<int, std::vector<PlotSample>> samples_;
    mutable std::map<int, double> estimated_inertias_;
};

struct ConsoleLine
{
    std::string text;
};

class LoggerThread
{
public:
    explicit LoggerThread(std::ofstream &file, bool enable_console = true, 
                         PlotterInterface *plotter = nullptr)
        : file_(file), enable_console_(enable_console), plotter_(plotter),
          running_(true)
    {
        thread_ = std::thread([this]() { run(); });
    }

    ~LoggerThread()
    {
        running_.store(false);
        if (thread_.joinable())
            thread_.join();
    }

    void enqueue_log(const LogEntry &entry)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        log_queue_.push(entry);
    }

    void enqueue_console(const std::string &text)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        console_queue_.push({text});
    }

    void wait_for_flush()
    {
        std::unique_lock<std::mutex> idle_lock(idle_mutex_);
        idle_cv_.wait(idle_lock, [this]() {
            std::lock_guard<std::mutex> queue_lock(queue_mutex_);
            return log_queue_.empty() && console_queue_.empty() && !plotter_redraw_ && idle_;
        });
    }

    void set_plotter_redraw(int iter, int freq = 5)
    {
        if (plotter_ && (iter % freq == 0))
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            plotter_redraw_ = true;
        }
    }

private:
    void run()
    {
        while (running_.load())
        {
            bool has_work = false;
            bool do_plot_redraw = false;
            std::vector<ConsoleLine> local_console;
            std::vector<LogEntry> local_logs;

            // Move queues into local containers while holding lock briefly
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!console_queue_.empty())
                {
                    local_console.push_back(console_queue_.front());
                    console_queue_.pop();
                }
                while (!log_queue_.empty())
                {
                    local_logs.push_back(log_queue_.front());
                    log_queue_.pop();
                }
                do_plot_redraw = plotter_redraw_;
                plotter_redraw_ = false;
            }

            // Process console messages (no mutex held)
            for (auto &msg : local_console)
            {
                if (enable_console_)
                    std::cout << msg.text;
                has_work = true;
            }

            // Process log entries (no mutex held)
            for (auto &entry : local_logs)
            {
                file_ << entry.timestamp_ms << ","
                      << entry.motor_id << ","
                      << entry.pos_raw << ","
                      << entry.target_deg << ","
                      << entry.target_vel_deg_s << ","
                      << entry.actual_deg << ","
                      << entry.error_deg << ","
                      << entry.vel_deg_s << ","
                      << entry.kp << ","
                      << entry.kd << ","
                      << entry.torque_ff << ","
                      << entry.actual_torque << ","
                      << entry.loop_dt_us << ","
                      << entry.observed_hz << "\n";
                // flush intermittently could be expensive; keep existing immediate flush
                file_.flush();

                // Store a copy for later aligned CSV export
                all_log_entries_.push_back(entry);

                if (plotter_)
                    plotter_->add_sample(entry);
                has_work = true;
            }

            // Handle plotter redraw (outside lock)
            if (do_plot_redraw && plotter_)
            {
                plotter_->redraw_all();
                has_work = true;
            }

            if (!has_work)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));

            {
                std::lock_guard<std::mutex> idle_lock(idle_mutex_);
                idle_ = !has_work;
            }
            idle_cv_.notify_all();
        }
    }

    std::ofstream &file_;
    bool enable_console_;
    PlotterInterface *plotter_;
    std::atomic<bool> running_;
    std::mutex idle_mutex_;
    std::condition_variable idle_cv_;
    bool idle_ = true;
    std::thread thread_;
    std::mutex queue_mutex_;
    std::queue<LogEntry> log_queue_;
    std::queue<ConsoleLine> console_queue_;
    bool plotter_redraw_ = false;
    std::vector<LogEntry> all_log_entries_;  // For aligned CSV export

public:
    void export_aligned_csv(const std::string &output_path) const
    {
        if (all_log_entries_.empty())
        {
            std::cerr << "No log entries to export.\n";
            return;
        }

        // Group entries by timestamp
        std::map<long, std::map<int, LogEntry>> timestamp_map;
        for (const auto &entry : all_log_entries_)
        {
            timestamp_map[entry.timestamp_ms][entry.motor_id] = entry;
        }

        // Collect all unique motor IDs
        std::set<int> motor_ids;
        for (const auto &entry : all_log_entries_)
            motor_ids.insert(entry.motor_id);

        // Write aligned CSV
        std::ofstream ofs(output_path);
        if (!ofs.is_open())
        {
            std::cerr << "Failed to open aligned CSV file: " << output_path << "\n";
            return;
        }

        // Write header
        ofs << "timestamp_ms";
        for (int mid : motor_ids)
        {
            ofs << ",actual_torque_m" << mid
                << ",vel_deg_s_m" << mid
                << ",actual_deg_m" << mid
                << ",target_deg_m" << mid
                << ",error_deg_m" << mid
                << ",torque_ff_m" << mid
                << ",kp_m" << mid
                << ",kd_m" << mid
                << ",loop_dt_us_m" << mid
                << ",observed_hz_m" << mid;
        }
        ofs << "\n";

        // Write data rows
        for (const auto &ts_entry : timestamp_map)
        {
            long ts = ts_entry.first;
            const auto &motors_at_ts = ts_entry.second;

            ofs << ts;
            for (int mid : motor_ids)
            {
                auto it = motors_at_ts.find(mid);
                if (it != motors_at_ts.end())
                {
                    const LogEntry &e = it->second;
                    ofs << "," << e.actual_torque
                        << "," << e.vel_deg_s
                        << "," << e.actual_deg
                        << "," << e.target_deg
                        << "," << e.error_deg
                        << "," << e.torque_ff
                        << "," << e.kp
                        << "," << e.kd
                        << "," << e.loop_dt_us
                        << "," << e.observed_hz;
                }
                else
                {
                    // Fill with empty values if motor not present at this timestamp
                    ofs << ",,,,,,,,,,";
                }
            }
            ofs << "\n";
        }

        ofs.close();
        std::cout << "✅ Exported aligned CSV: " << output_path << " (" 
                  << timestamp_map.size() << " timestamps, " 
                  << motor_ids.size() << " motors)\n";
    }
};

static std::string generate_timestamp_filename(const std::string &prefix, const std::string &suffix)
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    struct tm *tm_info = localtime(&time_t_now);
    
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", tm_info);
    
    std::ostringstream oss;
    oss << prefix << "_" << buffer << suffix;
    return oss.str();
}

void enable_realtime()
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        perror("mlockall");

    struct sched_param param;
    param.sched_priority = 90;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        perror("sched_setscheduler");

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
        perror("sched_setaffinity");
}

static bool is_motor_type(const std::string &value)
{
    return value == "rs00" || value == "rs01" || value == "rs02" ||
           value == "rs03" || value == "rs04";
}

static bool is_integer_token(const char *value)
{
    if (value == nullptr || *value == '\0')
        return false;

    char *end = nullptr;
    std::strtol(value, &end, 10);
    return end != value && *end == '\0';
}

static bool is_valid_can_interface(const std::string &iface)
{
    return iface.size() == 4 && iface.rfind("can", 0) == 0 &&
           iface[3] >= '0' && iface[3] <= '4';
}

static bool parse_can_interfaces(const std::string &arg, std::vector<std::string> &out)
{
    size_t start = 0;
    while (start < arg.size())
    {
        size_t comma = arg.find(',', start);
        std::string token = arg.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        if (token.empty() || !is_valid_can_interface(token))
            return false;
        if (std::find(out.begin(), out.end(), token) == out.end())
            out.push_back(token);
        if (comma == std::string::npos)
            break;
        start = comma + 1;
    }
    return !out.empty();
}

static void drain_can_buffers(std::vector<CanInterface> &buses)
{
    struct can_frame tmp;
    for (auto &bus : buses)
    {
        while (bus.read_frame(&tmp, 5))
        {
        }
    }
}

static int update_motor_from_frame(const struct can_frame &frame, std::vector<Motor> &motors)
{
    if (!(frame.can_id & CAN_EFF_FLAG))
        return -1;

    uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
    if (comm_type != CommType::OPERATION_STATUS)
        return -1;

    int src_id = (frame.can_id >> 8) & 0xFF;
    for (size_t idx = 0; idx < motors.size(); ++idx)
    {
        auto &m = motors[idx];
        if (src_id == m.id)
        {
            m.pos_raw = (uint16_t(frame.data[0]) << 8) | frame.data[1];
            uint16_t v_raw = (uint16_t(frame.data[2]) << 8) | frame.data[3];
            uint16_t t_raw = (uint16_t(frame.data[4]) << 8) | frame.data[5];
            read_status_scales(m.type, m.pos_raw, v_raw, t_raw, m.pos, m.vel, m.torque);
            if (!m.vel_filter_initialized)
            {
                m.filtered_vel = m.vel;
                m.vel_filter_initialized = true;
            }
            else
            {
                m.filtered_vel = VELOCITY_FILTER_ALPHA * m.vel +
                                 (1.0 - VELOCITY_FILTER_ALPHA) * m.filtered_vel;
            }
            return static_cast<int>(idx);
        }
    }
    return -1;
}

static void collect_status_all(std::vector<CanInterface> &buses, std::vector<Motor> &motors, int timeout_ms = 10)
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::vector<bool> seen(motors.size(), false);
    int received = 0;

    // Keep polling all buses until deadline or all motors seen
    while (received < (int)motors.size() && std::chrono::steady_clock::now() < deadline)
    {
        // Calculate remaining time
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0)
            break;

        // Read from each bus with a small blocking timeout
        int per_bus_timeout = std::max(1, (int)remaining.count() / ((int)buses.size() + 1));

        for (auto &bus : buses)
        {
            struct can_frame frame;
            // Do a blocking read with timeout for this bus, then drain any additional frames
            if (bus.read_frame(&frame, per_bus_timeout))
            {
                int updated_idx = update_motor_from_frame(frame, motors);
                if (updated_idx >= 0 && updated_idx < static_cast<int>(seen.size()) && !seen[updated_idx])
                {
                    seen[updated_idx] = true;
                    received++;
                }
                
                // Drain any additional pending frames from this bus (non-blocking)
                while (bus.read_frame(&frame, 0))
                {
                    updated_idx = update_motor_from_frame(frame, motors);
                    if (updated_idx >= 0 && updated_idx < static_cast<int>(seen.size()) && !seen[updated_idx])
                    {
                        seen[updated_idx] = true;
                        received++;
                    }
                }
            }
        }
    }
}

static void send_enable_all(std::vector<CanInterface> &buses, int motor_id)
{
    for (auto &bus : buses)
        send_enable(bus, motor_id);
}

static void send_mode_all(std::vector<CanInterface> &buses, int motor_id, int8_t mode)
{
    for (auto &bus : buses)
        send_mode(bus, motor_id, mode);
}

static void send_position_all(std::vector<CanInterface> &buses, int motor_id, double pos_rad,
                              double kp, double kd, const std::string &scale,
                              double torque_ff = 0.0, double vel_rad_s = 0.0)
{
    for (auto &bus : buses)
        send_position(bus, motor_id, pos_rad, kp, kd, scale, torque_ff, vel_rad_s);
}

static void set_zero_position_all(std::vector<CanInterface> &buses, uint8_t motor_id)
{
    for (auto &bus : buses)
        set_zero_position(bus, motor_id);
}

static void disable_all(std::vector<CanInterface> &buses, int motor_id)
{
    for (auto &bus : buses)
        disable(bus, motor_id);
}

int main(int argc, char *argv[])
{    std::ofstream file("motor_log.csv");

    if (!file.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return 1;
    }
    file << "timestamp_ms,"
     << "motor_id,"
     << "raw,"
     << "target_deg,"
     << "target_vel_deg_s,"
     << "actual_deg,"
     << "error_deg,"
     << "vel_deg_s,"
     << "kp,"
     << "kd,"
     << "torque_ff,"
     << "actual_torque,"
     << "loop_dt_us,"
     << "observed_hz"
     << "\n";

    int frequency_hz = 50;
    int arg_index = 1;
    int r_seconds = 10;
    int w_seconds = 1;
    int t_seconds = 10;
    std::vector<std::string> can_interfaces = {"can0"};
    enum OverrunPolicy { OP_WARN = 0, OP_CATCHUP = 1, OP_SKIP = 2 };
    OverrunPolicy overrun_policy = OP_WARN;

    // Parse leading --flags: --hz N, --r-sec N, --w-sec N, --t-sec N, --can can0[,can1,...], --overrun {warn,catchup,skip}
    while (arg_index < argc && std::string(argv[arg_index]).rfind("--", 0) == 0)
    {
        std::string flag = argv[arg_index];
        if (flag == "--hz" && arg_index + 1 < argc)
        {
            frequency_hz = std::max(1, std::atoi(argv[arg_index + 1]));
            arg_index += 2;
        }
        else if (flag == "--r-sec" && arg_index + 1 < argc)
        {
            r_seconds = std::max(0, std::atoi(argv[arg_index + 1]));
            arg_index += 2;
        }
        else if (flag == "--w-sec" && arg_index + 1 < argc)
        {
            w_seconds = std::max(0, std::atoi(argv[arg_index + 1]));
            arg_index += 2;
        }
        else if (flag == "--t-sec" && arg_index + 1 < argc)
        {
            t_seconds = std::max(0, std::atoi(argv[arg_index + 1]));
            arg_index += 2;
        }
        else if (flag == "--can" && arg_index + 1 < argc)
        {
            std::vector<std::string> parsed;
            if (!parse_can_interfaces(argv[arg_index + 1], parsed))
            {
                std::cerr << "Invalid CAN interface list: " << argv[arg_index + 1]
                          << ". Use can0..can4, comma-separated or repeated.\n";
                return -1;
            }
            for (auto &iface : parsed)
            {
                if (std::find(can_interfaces.begin(), can_interfaces.end(), iface) == can_interfaces.end())
                    can_interfaces.push_back(iface);
            }
            arg_index += 2;
        }
        else if (flag == "--overrun" && arg_index + 1 < argc)
        {
            std::string val = argv[arg_index + 1];
            if (val == "warn") overrun_policy = OP_WARN;
            else if (val == "catchup") overrun_policy = OP_CATCHUP;
            else if (val == "skip") overrun_policy = OP_SKIP;
            arg_index += 2;
        }
        else
        {
            break;
        }
    }

    if (argc - arg_index < 5)
    {
        std::cerr << "Usage: sudo ./multi_motor_functionality [--hz N] [--can can0[,can1,...]] <z|zp|r|w|t> <target_deg> <kp> <kd> [<id> <type> [<torque_nm> [<kp> <kd>]] ...]\n"
                  << "       sudo ./multi_motor_functionality [--hz N] [--can can0[,can1,...]] s <hold_ms> <cycles> <step_count> [<angle_deg> <kp> <kd> <torque_nm> <velocity_deg_s>]... [<id> <type> ...]\n"
                  << "  z   — zero all listed motors\n"
                  << "  zp  — zero only the motor whose id == target_deg\n"
                  << "  r   — passive read all (zero torque)\n"
                  << "  w   — move+read all to target_deg\n"
                  << "  s   — move+read all through step list, holding each target\n"
                  << "  torque_nm, kp, kd (optional per motor) — defaults are command kp/kd and zero torque\n"
                  << "Example: sudo ./multi_motor_functionality --hz 50 --can can0,can1 w 90 1.0 0.5 11 rs03 0.0 1.0 0.5 12 rs02 5.0 1.5 0.8\n"
                  << "Example: sudo ./multi_motor_functionality --hz 100 --can can0 --can can4 s 1000 1 2 0 1.0 0.5 0.0 0.0 90 1.2 0.6 0.0 20.0 11 rs03\n";
        return -1;
    }

    std::string cmd = argv[arg_index];
    double target_deg = 0.0;
    double target_rad = 0.0;
    double kp = 1.0;
    double kd = 0.5;
    int motor_arg_start = arg_index + 4;
    int hold_ms = 1000;
    int cycles = 1;
    bool sequence_has_step_controls = false;
    std::vector<SequenceStep> sequence_steps;

    auto is_motor_start_at = [&](int index) {
        return index + 1 < argc &&
               is_integer_token(argv[index]) &&
               is_motor_type(argv[index + 1]);
    };

    auto arg = [&](int relative) -> const char* {
        int index = arg_index + relative;
        return index < argc ? argv[index] : nullptr;
    };

    if (cmd == "s")
    {
        if (argc - arg_index < 5)
        {
            std::cerr << "Usage: sudo ./multi_motor_functionality [--hz N] s <hold_ms> <cycles> <step_count> [<angle_deg> <kp> <kd> <torque_nm> <velocity_deg_s>]... [<id> <type> ...]\n";
            return -1;
        }

        int step_count = std::max(0, std::atoi(arg(3)));
        int new_format_motor_arg_start = arg_index + 4 + step_count * 5;

        if (step_count > 0 && is_motor_start_at(new_format_motor_arg_start))
        {
            hold_ms = std::max(20, std::atoi(arg(1)));
            cycles = std::max(1, std::atoi(arg(2)));
            motor_arg_start = new_format_motor_arg_start;
            sequence_has_step_controls = true;

            for (int i = 0; i < step_count; i++)
            {
                int base = 4 + i * 5;
                SequenceStep step;
                step.angle_deg = std::atof(arg(base));
                step.kp = std::atof(arg(base + 1));
                step.kd = std::atof(arg(base + 2));
                step.torque_ff = std::atof(arg(base + 3));
                step.velocity_deg_s = std::atof(arg(base + 4));
                sequence_steps.push_back(step);
            }

            kp = sequence_steps.front().kp;
            kd = sequence_steps.front().kd;
        }
        else
        {
            kp = std::atof(arg(1));
            kd = std::atof(arg(2));
            hold_ms = std::max(20, std::atoi(arg(3)));
            cycles = std::max(1, std::atoi(arg(4)));
            int angle_count = std::max(0, std::atoi(arg(5)));
            motor_arg_start = arg_index + 6 + angle_count;

            if (angle_count <= 0 || !is_motor_start_at(motor_arg_start))
            {
                std::cerr << "Sequence command needs at least one step and one motor.\n";
                return -1;
            }

            for (int i = 0; i < angle_count; i++)
            {
                SequenceStep step;
                step.angle_deg = std::atof(arg(6 + i));
                step.kp = kp;
                step.kd = kd;
                sequence_steps.push_back(step);
            }
        }
    }
    else
    {
        target_deg = std::atof(arg(1));
        target_rad = target_deg * M_PI / 180.0;
        kp = std::atof(arg(2));
        kd = std::atof(arg(3));
    }

    std::vector<Motor> motors;
    // Parse motors: [<id> <type> [<torque> [<kp> <kd>]]] ...
    for (int i = motor_arg_start; i + 1 < argc;)
    {
        Motor m;
        m.id = std::atoi(argv[i]);
        m.type = argv[i + 1];
        m.target_rad = target_rad;
        m.kp = kp;
        m.kd = kd;

        i += 2;

        if (i < argc && !is_motor_start_at(i))
        {
            m.torque_ff = std::atof(argv[i]);
            i++;
        }
        if (i < argc && !is_motor_start_at(i))
        {
            m.kp = std::atof(argv[i]);
            i++;
        }
        if (i < argc && !is_motor_start_at(i))
        {
            m.kd = std::atof(argv[i]);
            i++;
        }
        motors.push_back(m);
    }

    std::vector<CanInterface> can_buses;
    std::cerr << "Initializing " << can_interfaces.size() << " CAN interface(s): ";
    for (const auto &iface : can_interfaces)
        std::cerr << iface << " ";
    std::cerr << "\n";
    
    for (const auto &iface : can_interfaces)
    {
        can_buses.emplace_back();
        std::cerr << "  Initializing " << iface << "... ";
        if (!can_buses.back().init(iface) || !can_buses.back().is_ready())
        {
            std::cerr << "FAILED\n";
            return -1;
        }
        std::cerr << "OK\n";
    }
    std::cerr << "All " << can_buses.size() << " CAN interface(s) ready.\n";

    // ---- zero all -------------------------------------------------------
    if (cmd == "z")
    {
        for (auto &m : motors)
        {
            std::cout << "Zeroing motor " << m.id << " (" << m.type << ")...\n";
            set_zero_position_all(can_buses, m.id);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        drain_can_buffers(can_buses);
        std::cout << "Done. All motors zeroed.\n";
        return 0;
    }

    // ---- zero particular ------------------------------------------------
    if (cmd == "zp")
    {
        int target_id = (int)target_deg; // reuse target_deg slot as motor id
        bool found = false;
        for (auto &m : motors)
        {
            if (m.id == target_id)
            {
                std::cout << "Zeroing motor " << m.id << " (" << m.type << ")...\n";
                set_zero_position_all(can_buses, m.id);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                found = true;
                break;
            }
        }
        if (!found)
            std::cerr << "Motor id " << target_id << " not found in list.\n";
        drain_can_buffers(can_buses);
        return 0;
    }

    // ---- enable + MIT mode for all --------------------------------------
    for (auto &m : motors)
        send_enable_all(can_buses, m.id);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (auto &m : motors)
        send_mode_all(can_buses, m.id, ControlMode::MIT_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    enable_realtime();
    auto start_time = std::chrono::steady_clock::now();
    double loop_period_ms = 1000.0 / frequency_hz;
    auto loop_period = std::chrono::duration<double, std::milli>(loop_period_ms);

    // ---- read all (passive, zero torque) --------------------------------
    if (cmd == "r")
    {
        LoggerThread logger(file, true);
        auto stop_time = std::chrono::steady_clock::now() + std::chrono::seconds(r_seconds);
        auto next_time = std::chrono::steady_clock::now();
        auto last_cycle_time = std::chrono::steady_clock::now();
        bool first_cycle = true;
        while (std::chrono::steady_clock::now() < stop_time)
        {
            auto loop_start = std::chrono::steady_clock::now();
            double dt_us = 0.0;
            double observed_hz = 0.0;
            if (!first_cycle)
            {
                dt_us = std::chrono::duration<double, std::micro>(loop_start - last_cycle_time).count();
                observed_hz = dt_us > 0.0 ? 1e6 / dt_us : 0.0;
            }
            first_cycle = false;
            last_cycle_time = loop_start;
            for (auto &m : motors)
            {
                uint8_t data[8];
                pack_u16_be(&data[0], 0x7FFF);
                pack_u16_be(&data[2], 0x7FFF);
                pack_u16_be(&data[4], 0x0000); // kp=0
                pack_u16_be(&data[6], 0x0000); // kd=0
                uint32_t ctrl_id = (CommType::OPERATION_CONTROL << 24) | (0x7FFF << 8) | m.id;
                for (auto &bus : can_buses)
                    bus.write_frame(ctrl_id, data, 8);
            }
            collect_status_all(can_buses, motors);
            
            auto now_stamp = std::chrono::steady_clock::now();
            auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_stamp - start_time).count();

            // Enqueue logs instead of blocking on I/O
            for (auto &m : motors)
            {
                std::ostringstream oss;
                oss << "motor=" << m.id
                    << "  raw=" << m.pos_raw
                    << "  pos=" << m.pos * 180.0 / M_PI << " deg"
                    << "  vel=" << m.filtered_vel * 180.0 / M_PI << " deg/s"
                    << "  torque=" << m.torque << " Nm\n";
                logger.enqueue_console(oss.str());

                LogEntry entry;
                entry.timestamp_ms = timestamp_ms;
                entry.motor_id = m.id;
                entry.pos_raw = m.pos_raw;
                entry.target_deg = 0.0;
                entry.target_vel_deg_s = 0.0;
                entry.actual_deg = m.pos * 180.0 / M_PI;
                entry.error_deg = -entry.actual_deg;
                entry.vel_deg_s = m.filtered_vel * 180.0 / M_PI;
                entry.kp = 0.0;
                entry.kd = 0.0;
                entry.torque_ff = 0.0;
                entry.actual_torque = m.torque;
                entry.loop_dt_us = dt_us;
                entry.observed_hz = observed_hz;
                logger.enqueue_log(entry);
            }

            next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
            auto now = std::chrono::steady_clock::now();
            if (now < next_time)
                std::this_thread::sleep_until(next_time);
            else
            {
                if (overrun_policy == OP_WARN)
                {
                    std::cerr << "Warning: loop overrun in 'r' mode\n";
                    next_time = std::chrono::steady_clock::now();
                }
                else if (overrun_policy == OP_SKIP)
                {
                    next_time = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                }
                else if (overrun_policy == OP_CATCHUP)
                {
                    while (next_time <= now)
                        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                }
            }
        }
        logger.wait_for_flush();
        std::string aligned_csv_r = generate_timestamp_filename("motor_log_aligned", ".csv");
        logger.export_aligned_csv(aligned_csv_r);
        for (auto &m : motors)
            disable_all(can_buses, m.id);
        return 0;
    }

    // ---- move + read all ------------------------------------------------
    if (cmd == "w")
    {
        LoggerThread logger(file, true);
        auto stop_time = std::chrono::steady_clock::now() + std::chrono::seconds(w_seconds);
        auto next_time = std::chrono::steady_clock::now();
        auto last_cycle_time = std::chrono::steady_clock::now();
        bool first_cycle = true;
        while (std::chrono::steady_clock::now() < stop_time)
        {
            auto loop_start = std::chrono::steady_clock::now();
            double dt_us = 0.0;
            double observed_hz = 0.0;
            if (!first_cycle)
            {
                dt_us = std::chrono::duration<double, std::micro>(loop_start - last_cycle_time).count();
                observed_hz = dt_us > 0.0 ? 1e6 / dt_us : 0.0;
            }
            first_cycle = false;
            last_cycle_time = loop_start;
            for (auto &m : motors)
                send_position_all(can_buses, m.id, m.target_rad, m.kp, m.kd, m.type, m.torque_ff);
            collect_status_all(can_buses, motors);
            auto now_stamp = std::chrono::steady_clock::now();

            for (auto &m : motors)
            {
                double actual_deg = m.pos * 180.0 / M_PI;
                double vel_deg_s = m.filtered_vel * 180.0 / M_PI;
                double motor_target_deg = m.target_rad * 180.0 / M_PI;
                double error_deg = motor_target_deg - actual_deg;
                
                std::ostringstream oss;
                oss << "motor=" << m.id
                    << "  raw=" << m.pos_raw
                    << "  pos=" << actual_deg << " deg"
                    << "  vel=" << vel_deg_s << " deg/s"
                    << "  target=" << motor_target_deg << " deg"
                    << "  torque=" << m.torque << " Nm\n";
                logger.enqueue_console(oss.str());

                auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_stamp - start_time).count();
                LogEntry entry;
                entry.timestamp_ms = timestamp_ms;
                entry.motor_id = m.id;
                entry.pos_raw = m.pos_raw;
                entry.target_deg = motor_target_deg;
                entry.target_vel_deg_s = 0.0;
                entry.actual_deg = actual_deg;
                entry.error_deg = error_deg;
                entry.vel_deg_s = vel_deg_s;
                entry.kp = m.kp;
                entry.kd = m.kd;
                entry.torque_ff = m.torque_ff;
                entry.actual_torque = m.torque;
                entry.loop_dt_us = dt_us;
                entry.observed_hz = observed_hz;
                logger.enqueue_log(entry);
            }

            next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
            auto now = std::chrono::steady_clock::now();
            if (now < next_time)
                std::this_thread::sleep_until(next_time);
            else
            {
                if (overrun_policy == OP_WARN)
                {
                    std::cerr << "Warning: loop overrun in 'w' mode\n";
                    next_time = std::chrono::steady_clock::now();
                }
                else if (overrun_policy == OP_SKIP)
                {
                    next_time = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                }
                else if (overrun_policy == OP_CATCHUP)
                {
                    while (next_time <= now)
                        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                }
            }
        }
        logger.wait_for_flush();
        std::string aligned_csv_w = generate_timestamp_filename("motor_log_aligned", ".csv");
        logger.export_aligned_csv(aligned_csv_w);
        std::cout << "CSV logging completed." << std::endl;
        for (auto &m : motors)
            disable_all(can_buses, m.id);
        return 0;
    }

    // ---- angle sequence + hold -----------------------------------------
    if (cmd == "s")
    {
        RealtimeGnuplot plotter(motors);
        LoggerThread logger(file, true, &plotter);
        int plot_iter = 0;

        for (int cycle = 0; cycle < cycles; cycle++)
        {
            for (const auto &step : sequence_steps)
            {
                double angle_rad = step.angle_deg * M_PI / 180.0;
                double velocity_rad_s = step.velocity_deg_s * M_PI / 180.0;
                for (auto &m : motors)
                    m.target_rad = angle_rad;

                std::ostringstream header;
                header << "Holding target " << step.angle_deg << " deg";
                if (sequence_has_step_controls)
                    header << "  kp=" << step.kp
                           << "  kd=" << step.kd
                           << "  torque=" << step.torque_ff << " Nm"
                           << "  velocity=" << step.velocity_deg_s << " deg/s";
                header << "  cycle=" << (cycle + 1) << "/" << cycles << "\n";
                logger.enqueue_console(header.str());

                auto hold_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms);
                auto next_time = std::chrono::steady_clock::now();
                auto last_cycle_time = std::chrono::steady_clock::now();
                bool first_cycle = true;
                while (std::chrono::steady_clock::now() < hold_end)
                {
                    auto loop_start = std::chrono::steady_clock::now();
                    double dt_us = 0.0;
                    double observed_hz = 0.0;
                    if (!first_cycle)
                    {
                        dt_us = std::chrono::duration<double, std::micro>(loop_start - last_cycle_time).count();
                        observed_hz = dt_us > 0.0 ? 1e6 / dt_us : 0.0;
                    }
                    first_cycle = false;
                    last_cycle_time = loop_start;
                    for (auto &m : motors)
                    {
                        double command_kp = sequence_has_step_controls ? step.kp : m.kp;
                        double command_kd = sequence_has_step_controls ? step.kd : m.kd;
                        double command_torque = sequence_has_step_controls ? step.torque_ff : m.torque_ff;
                        double command_velocity = sequence_has_step_controls ? velocity_rad_s : 0.0;
                        send_position_all(can_buses, m.id, m.target_rad, command_kp, command_kd,
                                          m.type, command_torque, command_velocity);
                    }

                    collect_status_all(can_buses, motors);
                    auto now_stamp = std::chrono::steady_clock::now();
                    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_stamp - start_time).count();

                    for (auto &m : motors)
                    {
                        double actual_deg = m.pos * 180.0 / M_PI;
                        double vel_deg_s = m.filtered_vel * 180.0 / M_PI;
                        double error_deg = step.angle_deg - actual_deg;
                        double command_kp = sequence_has_step_controls ? step.kp : m.kp;
                        double command_kd = sequence_has_step_controls ? step.kd : m.kd;
                        double command_torque = sequence_has_step_controls ? step.torque_ff : m.torque_ff;
                        double command_velocity_deg_s = sequence_has_step_controls ? step.velocity_deg_s : 0.0;

                        std::ostringstream oss;
                        oss << "motor=" << m.id
                            << "  raw=" << m.pos_raw
                            << "  pos=" << actual_deg << " deg"
                            << "  vel=" << vel_deg_s << " deg/s"
                            << "  target=" << step.angle_deg << " deg"
                            << "  torque=" << m.torque << " Nm\n";
                        logger.enqueue_console(oss.str());

                        LogEntry entry;
                        entry.timestamp_ms = timestamp_ms;
                        entry.motor_id = m.id;
                        entry.pos_raw = m.pos_raw;
                        entry.target_deg = step.angle_deg;
                        entry.target_vel_deg_s = command_velocity_deg_s;
                        entry.actual_deg = actual_deg;
                        entry.error_deg = error_deg;
                        entry.vel_deg_s = vel_deg_s;
                        entry.kp = command_kp;
                        entry.kd = command_kd;
                        entry.torque_ff = command_torque;
                        entry.actual_torque = m.torque;
                        entry.loop_dt_us = dt_us;
                        entry.observed_hz = observed_hz;
                        logger.enqueue_log(entry);
                    }

                    logger.set_plotter_redraw(plot_iter++, 100);
                    next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                    auto now = std::chrono::steady_clock::now();
                    if (now < next_time)
                        std::this_thread::sleep_until(next_time);
                    else
                    {
                        if (overrun_policy == OP_WARN)
                        {
                            std::cerr << "Warning: loop overrun in 's' sequence hold\n";
                            next_time = std::chrono::steady_clock::now();
                        }
                        else if (overrun_policy == OP_SKIP)
                        {
                            next_time = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                        }
                        else if (overrun_policy == OP_CATCHUP)
                        {
                            while (next_time <= now)
                                next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                        }
                    }
                }
            }
        }

        logger.wait_for_flush();
        std::string aligned_csv_s = generate_timestamp_filename("motor_log_aligned", ".csv");
        logger.export_aligned_csv(aligned_csv_s);
        if (plotter.ready())
        {
            plotter.redraw_all();
        }

        std::cout << "CSV logging completed." << std::endl;
        for (auto &m : motors)
            disable_all(can_buses, m.id);
        return 0;
    }
    // ===================================================================
    // EQUATIONS USED IN TORQUE MODE (MULTI-SINE EXCITATION)
    // ===================================================================

    // 1. Main Multi-Sine Torque Command
    // torque_cmd(t) = A * sum[ sin(2π * fi * t) ]  for i = 0 to N-1

    // 2. Frequency for each component
    // fi = base_freq * ratio[i % 3]
    // where:
    //     base_freq = 5.0 Hz
    //     ratio = [1.0, 3.4, 7.4]

    // Example frequencies (repeating):
    // 5.0 * 1.0 = 5.0 Hz
    // 5.0 * 3.4 = 17.0 Hz  
    // 5.0 * 7.4 = 37.0 Hz

    // 3. Final Clamped Torque Command
    // torque_cmd = clamp( torque_cmd, -1.1 * A, +1.1 * A )

    // Full combined equation:
    // torque_cmd(t) = clamp( A * Σ sin(2π * fi * t), -1.1*A, +1.1*A )
    // ---- torque control + read all (pure torque, zero position/gains) ---
    if (cmd == "t")
    {
        RealtimeTorquePlotter plotter(motors);
        LoggerThread logger(file, true, &plotter);
        int plot_iter = 0;

        auto stop_time = std::chrono::steady_clock::now() + std::chrono::seconds(t_seconds);
        auto next_time = std::chrono::steady_clock::now();
        auto last_cycle_time = std::chrono::steady_clock::now();
        bool first_cycle = true;
        double amplitude = motors.empty() ? 0.0 : motors.front().torque_ff;
        double base_freq = 1.0; // Hz  (observie's base f)
        std::vector<double> ratio  = { 1.0, 3.4, 7.4 };   // irrational ratios — no cancellation
        std::vector<double> weight = { 1.0, 0.6, 0.3 };   // decreasing amplitude (observie multi-sine)
        int num_components = ratio.size();
        std::vector<double> fi(num_components, 0.0);

        // Initialize frequencies (do this once, not repeatedly)
        for(int i = 0; i < num_components; i++)
        {
            fi[i] = base_freq * ratio[i % ratio.size()];
        }

        while (std::chrono::steady_clock::now() < stop_time)
        {
            auto loop_start = std::chrono::steady_clock::now();
            double dt_us = 0.0;
            double observed_hz = 0.0;

            if (!first_cycle)
            {
                dt_us = std::chrono::duration<double, std::micro>(loop_start - last_cycle_time).count();
                observed_hz = dt_us > 0.0 ? 1e6 / dt_us : 0.0;
            }
            first_cycle = false;
            last_cycle_time = loop_start;

            // Calculate time once per cycle
            double t = std::chrono::duration<double>(loop_start - start_time).count();

                    // Compute multi-sine torque — RMS = amplitude
            double torque_cmd = 0.0;
            for (int i = 0; i < num_components; i++)
                torque_cmd += weight[i] * std::sin(2.0 * M_PI * fi[i] * t);

            // observie literal: torque(t) = amp * (sin + 0.6 sin + 0.3 sin).
            // 'amplitude' (the CLI torque arg) IS observie's amp; signal peak ~= 1.9*amp.
            // No clamping -> peaks are never clipped.
            torque_cmd *= amplitude;

            // Clamp at ±amplitude so peak never exceeds the specified value
      //      torque_cmd = std::max(-amplitude, std::min(amplitude, torque_cmd));

            // Send torque command to all motors once
            for (auto &m : motors)
                    send_position_all(can_buses, m.id, 0.0, 0.0, 0.0, m.type, torque_cmd);

            // Collect status once per cycle
            collect_status_all(can_buses, motors);
            auto now_stamp = std::chrono::steady_clock::now();
            auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_stamp - start_time).count();

            for (auto &m : motors)
            {
                std::ostringstream oss;
                oss << "motor=" << m.id
                    << "  raw=" << m.pos_raw
                    << "  pos=" << m.pos * 180.0 / M_PI << " deg"
                    << "  vel=" << m.filtered_vel * 180.0 / M_PI << " deg/s"
                    << "  torque=" << m.torque << " Nm\n";
                logger.enqueue_console(oss.str());

                LogEntry entry;
                entry.timestamp_ms = timestamp_ms;
                entry.motor_id = m.id;
                entry.pos_raw = m.pos_raw;
                entry.target_deg = 0.0;
                entry.target_vel_deg_s = 0.0;
                entry.actual_deg = m.pos * 180.0 / M_PI;
                entry.error_deg = -entry.actual_deg;
                entry.vel_deg_s = m.filtered_vel * 180.0 / M_PI;
                entry.kp = 0.0;
                entry.kd = 0.0;
                entry.torque_ff = torque_cmd;
                entry.actual_torque = m.torque;
                entry.loop_dt_us = dt_us;
                entry.observed_hz = observed_hz;
                logger.enqueue_log(entry);
            }

            logger.set_plotter_redraw(plot_iter++, 100);
            next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
            auto now = std::chrono::steady_clock::now();
            if (now < next_time)
                std::this_thread::sleep_until(next_time);
            else
            {
                if (overrun_policy == OP_WARN)
                {
                    std::cerr << "Warning: loop overrun in 't' mode\n";
                    next_time = std::chrono::steady_clock::now();
                }
                else if (overrun_policy == OP_SKIP)
                {
                    next_time = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                }
                else if (overrun_policy == OP_CATCHUP)
                {
                    while (next_time <= now)
                        next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(loop_period);
                }
            }
        }
        logger.wait_for_flush();
        std::string aligned_csv_t = generate_timestamp_filename("motor_log_aligned", ".csv");
        logger.export_aligned_csv(aligned_csv_t);
        if (plotter.ready())
        {
            plotter.redraw_all();
        }
        std::cout << "CSV logging completed." << std::endl;
        for (auto &m : motors)
            disable_all(can_buses, m.id);
        return 0;
    }

    std::cerr << "Unknown command: " << cmd << "\n";
    return -1;
}
