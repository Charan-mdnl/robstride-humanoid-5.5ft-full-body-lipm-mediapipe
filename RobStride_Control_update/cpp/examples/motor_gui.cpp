/*
 * motor_gui.cpp — Dear ImGui GUI for selecting motors and commands.
 *
 * Displays a window where the user can:
 *   - Add/remove motors (ID + model type)
 *   - Choose a command: Zero, Read, or Move+Read
 *   - Set a target angle (degrees)
 *   - Click "Run" to print the constructed command to stdout (cout)
 *
 * Build via CMake (see CMakeLists.txt).
 * Run button executes the command in a background thread (GUI stays live).
 */

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <algorithm>
#include <cstdlib>
#include <cmath>

// ---- data types -----------------------------------------------------------

static const char* TYPE_NAMES[] = {"rs00", "rs01", "rs02", "rs03", "rs04"};
static const int   TYPE_COUNT   = 5;

struct MotorEntry {
    int id   = 1;
    int type = 3;   // default rs03
    float torque_ff = 0.0f;  // feedforward torque in Nm
    double kp = 1.0;          // position gain
    double kd = 0.5;          // derivative gain
};

struct SequenceStep {
    float angle_deg = 0.0f;
    double kp = 1.0;
    double kd = 0.5;
    float torque_ff = 0.0f;
    float velocity_deg_s = 0.0f;
};

// ---- helpers --------------------------------------------------------------

// cmd indices: 0=z, 1=zp, 2=r, 3=w, 4=t, 5=s
static const char* CMD_CHARS[] = {"z", "zp", "r", "w", "t", "s"};

static std::string build_can_interface_string(const char* can_ifaces[], const bool selected[], int count)
{
    std::ostringstream oss;
    bool first = true;
    for (int i = 0; i < count; ++i)
    {
        if (!selected[i])
            continue;
        if (!first)
            oss << ',';
        oss << can_ifaces[i];
        first = false;
    }
    return oss.str();
}

static std::string build_command(int frequency_hz, int cmd, float target_deg, float torque_val, int zero_one_id,
                                 const std::vector<MotorEntry>& motors,
                                 const std::vector<SequenceStep>& sequence_steps,
                                 int hold_ms, int sequence_cycles,
                                 int r_sec, int w_sec, int t_sec,
                                 const std::string &overrun,
                                 const std::string &can_interface)
{
    std::ostringstream oss;
    oss << "./multi_motor_functionality --hz " << frequency_hz
        << " --can " << can_interface
        << " --r-sec " << r_sec
        << " --w-sec " << w_sec
        << " --t-sec " << t_sec
        << " --overrun " << overrun
        << " " << CMD_CHARS[cmd];
    
    // Non-sequence commands keep the older target/kp/kd argument layout.
    double global_kp = motors.empty() ? 1.0 : motors[0].kp;
    double global_kd = motors.empty() ? 0.5 : motors[0].kd;
    
    if (cmd == 1) {
        // 'zp': zero one motor
        oss << " " << zero_one_id << " " << global_kp << " " << global_kd;
    } else if (cmd == 4) {
        // 't': torque control
        oss << " 0.0 " << global_kp << " " << global_kd;
    } else if (cmd == 5) {
        // 's': sequence control: hold_ms cycles step_count [angle kp kd torque velocity]...
        oss << " " << hold_ms
            << " " << sequence_cycles
            << " " << sequence_steps.size();
        for (const auto& step : sequence_steps) {
            oss << " " << step.angle_deg
                << " " << step.kp
                << " " << step.kd
                << " " << step.torque_ff
                << " " << step.velocity_deg_s;
        }
    } else {
        // 'z', 'r', 'w': use target_deg
        oss << " " << (int)target_deg << " " << global_kp << " " << global_kd;
    }
    
    // Append each motor: id type torque kp kd
    for (const auto& m : motors) {
        if (cmd == 5) {
            oss << " " << m.id << " " << TYPE_NAMES[m.type];
            continue;
        }

        float torque = (cmd == 3 || cmd == 4 || cmd == 5) ? torque_val : m.torque_ff;
        oss << " " << m.id << " " << TYPE_NAMES[m.type]
            << " " << torque
            << " " << m.kp
            << " " << m.kd;
    }
    
    return oss.str();
}

// ---- main -----------------------------------------------------------------

int main(int, char**)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init error: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow(
        "RobStride Motor Control",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        640, 520,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

    if (!window) {
        std::cerr << "SDL_CreateWindow error: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;   // don't save layout to disk

    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 130");

    // ---- state ------------------------------------------------------------
    std::vector<MotorEntry> motors;
    int   cmd        = 3;       // 0=z, 1=zp, 2=r, 3=w, 4=t, 5=s
    float target_deg = 90.0f;
    float torque_val = 0.0f;    // torque for 't' command
    int   zero_one_id = 1;      // motor id to zero for 'zp'
    int   new_id     = 1;
    double kp = 1.0;
    double kd = 0.5;
    int   new_type   = 3;       // rs03
    std::vector<SequenceStep> sequence_steps = {
        {0.0f, 1.0, 0.5, 0.0f, 0.0f},
        {45.0f, 1.0, 0.5, 0.0f, 0.0f},
        {90.0f, 1.0, 0.5, 0.0f, 0.0f},
    };
    SequenceStep new_sequence_step;
    int hold_ms = 1000;
    int sequence_cycles = 1;
    int control_frequency_hz = 1000;
    int run_r_sec = 10;
    int run_w_sec = 1;
    int run_t_sec = 10;
    const char* can_ifaces[] = {"can0", "can1", "can2", "can3", "can4"};
    bool can_iface_selected[5] = {true, false, false, false, false};
    const char* overrun_opts[] = {"warn", "catchup", "skip"};
    int overrun_idx = 0;
    bool  running    = true;
    std::atomic<bool> running_cmd{false};
    
    // Torque calculator state
    float calc_mass = 1.0f;           // kg
    float calc_length = 0.5f;         // meters
    const float g_accel = 9.81f;      // m/s^2 (gravity)

    // ---- main loop --------------------------------------------------------
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) running = false;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Full-window panel
        ImGui::SetNextWindowPos({0, 0});
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("##root", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

        // ---- command selection -------------------------------------------
        ImGui::SeparatorText("Command");
        ImGui::RadioButton("Zero all (z)",    &cmd, 0); ImGui::SameLine();
        ImGui::RadioButton("Zero one (zp)",   &cmd, 1); ImGui::SameLine();
        ImGui::RadioButton("Read (r)",         &cmd, 2); ImGui::SameLine();
        ImGui::RadioButton("Move+Read (w)",    &cmd, 3); ImGui::SameLine();
        ImGui::RadioButton("Torque (t)",       &cmd, 4); ImGui::SameLine();
        ImGui::RadioButton("Angle Sequence (s)", &cmd, 5);

        ImGui::Spacing();
        ImGui::SeparatorText("Control Loop");
        ImGui::SetNextItemWidth(300);
        ImGui::SliderInt("Frequency (Hz)##freq", &control_frequency_hz, 1, 1000);
        control_frequency_hz = std::max(1, std::min(1000, control_frequency_hz));
        ImGui::Text("Period = %.2f ms", 1000.0 / control_frequency_hz);
        ImGui::Spacing();
        ImGui::Text("CAN Interfaces");
        for (int i = 0; i < 5; ++i)
        {
            if (i > 0)
                ImGui::SameLine();
            ImGui::Checkbox(can_ifaces[i], &can_iface_selected[i]);
        }
        bool any_can_selected = false;
        for (int i = 0; i < 5; ++i)
            any_can_selected |= can_iface_selected[i];
        if (!any_can_selected)
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "Select at least one CAN interface.");
        ImGui::Spacing();
        ImGui::SetNextItemWidth(150);
        ImGui::InputInt("Read run (s)##rsec", &run_r_sec);
        run_r_sec = std::max(0, run_r_sec);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::InputInt("Move run (s)##wsec", &run_w_sec);
        run_w_sec = std::max(0, run_w_sec);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::InputInt("Torque run (s)##tsec", &run_t_sec);
        run_t_sec = std::max(0, run_t_sec);
        ImGui::Spacing();
        ImGui::SetNextItemWidth(200);
        ImGui::Combo("Overrun Policy", &overrun_idx, overrun_opts, IM_ARRAYSIZE(overrun_opts));
        ImGui::Spacing();
        ImGui::Separator();

        // ---- torque calculator ----
        ImGui::SeparatorText("Torque Calculator (τ = m·g·r·sin(θ))");
        ImGui::SetNextItemWidth(150);
        ImGui::InputFloat("Mass (kg)##calc", &calc_mass, 0.1f, 1.0f, "%.2f");
        calc_mass = std::max(0.01f, calc_mass);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::InputFloat("Length (m)##calc", &calc_length, 0.1f, 0.5f, "%.3f");
        calc_length = std::max(0.001f, calc_length);
        ImGui::SetNextItemWidth(100);
        static float calc_angle = 0.0f;
        ImGui::InputFloat("Angle (deg)##calc", &calc_angle, 1.0f, 10.0f, "%.1f");
        
        float angle_rad = calc_angle * 3.14159265f / 180.0f;
        float calculated_torque = calc_mass * g_accel * calc_length * std::sin(angle_rad);
        ImGui::Text("Calculated Torque: %.3f Nm", calculated_torque);
        
        if (ImGui::Button("Copy to Move Torque##calc", {180, 0})) {
            torque_val = calculated_torque;
        }
        ImGui::SameLine();
        if (ImGui::Button("Copy to Torque Control##calc", {180, 0})) {
            torque_val = calculated_torque;
        }
        ImGui::Spacing();
        ImGui::Separator();

        // ---- command parameters -------------------------------------------
        if (cmd == 1) {
            ImGui::SeparatorText("Zero One Motor");
            ImGui::SetNextItemWidth(150);
            ImGui::InputInt("Motor ID to zero##zp", &zero_one_id);
            zero_one_id = std::max(1, std::min(255, zero_one_id));
        } else if (cmd == 3) {
            ImGui::SeparatorText("Move+Read Parameters");
            ImGui::SetNextItemWidth(300);
            ImGui::SliderFloat("Target (deg)##w", &target_deg, -720.0f, 720.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("##target_input_w", &target_deg, 1.0f, 10.0f, "%.1f");
            ImGui::SetNextItemWidth(120);
            ImGui::InputDouble("Kp (Nm/rad)##w", &kp, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::InputDouble("Kd (Nm·s/rad)##w", &kd, 0.1, 1.0, "%.2f");
            ImGui::SetNextItemWidth(150);
            ImGui::InputFloat("Feedforward Torque##w", &torque_val, 0.1f, 1.0f, "%.2f");
        } else if (cmd == 4) {
            ImGui::SeparatorText("Torque Control Parameters");
            ImGui::SetNextItemWidth(300);
            ImGui::SliderFloat("Torque (Nm)##t", &torque_val, -60.0f, 60.0f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("##torque_input_t", &torque_val, 1.0f, 5.0f, "%.2f");
            ImGui::SetNextItemWidth(120);
            ImGui::InputDouble("Kp (Nm/rad)##t", &kp, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::InputDouble("Kd (Nm·s/rad)##t", &kd, 0.1, 1.0, "%.2f");
        } else if (cmd == 5) {
            ImGui::SeparatorText("Angle Sequence Parameters");
            ImGui::SetNextItemWidth(140);
            ImGui::InputInt("Hold time (ms)##s", &hold_ms, 100, 500);
            hold_ms = std::max(20, hold_ms);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::InputInt("Cycles##s", &sequence_cycles, 1, 5);
            sequence_cycles = std::max(1, sequence_cycles);

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::SeparatorText("Generate from Range");
            static float range_start = 0.0f, range_stop = 90.0f, range_step = 10.0f;
            static double range_kp_start = 1.0, range_kp_stop = 1.0, range_kp_step = 0.1;
            static double range_kd_start = 0.5, range_kd_stop = 0.5, range_kd_step = 0.1;
            static float range_torque_start = 0.0f, range_torque_stop = 0.0f, range_torque_step = 1.0f;
            static float range_velocity_start = 0.0f, range_velocity_stop = 0.0f, range_velocity_step = 1.0f;
            ImGui::SetNextItemWidth(105);
            ImGui::InputFloat("Start (deg)##range", &range_start, 1.0f, 10.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(105);
            ImGui::InputFloat("Stop (deg)##range", &range_stop, 1.0f, 10.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(105);
            ImGui::InputFloat("Step (deg)##range", &range_step, 1.0f, 10.0f, "%.1f");

            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kp start##range", &range_kp_start, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kp stop##range", &range_kp_stop, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kp step##range", &range_kp_step, 0.1, 1.0, "%.2f");

            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kd start##range", &range_kd_start, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kd stop##range", &range_kd_stop, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kd step##range", &range_kd_step, 0.1, 1.0, "%.2f");

            ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "Torque auto-calculated from τ = m·g·r·sin(θ)");
            ImGui::SetNextItemWidth(95);
            ImGui::InputFloat("Torque scalar##range", &range_torque_start, 0.1f, 1.0f, "%.2f");
            ImGui::SameLine();
            ImGui::TextDisabled("(multiplier, 0=disabled)");

            ImGui::SetNextItemWidth(95);
            ImGui::InputFloat("Velocity start##range", &range_velocity_start, 1.0f, 10.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputFloat("Velocity stop##range", &range_velocity_stop, 1.0f, 10.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputFloat("Velocity step##range", &range_velocity_step, 1.0f, 10.0f, "%.1f");

            ImGui::SameLine();
            if (ImGui::Button("Generate##range", {100, 0})) {
                auto make_float_range = [&](float start, float stop, float step) {
                    std::vector<float> out;
                    if (step <= 0.0f || start > stop) {
                        out.push_back(start);
                        return out;
                    }
                    for (float value = start; value <= stop + 1e-6f; value += step)
                        out.push_back(value);
                    if (out.empty())
                        out.push_back(start);
                    return out;
                };
                auto make_double_range = [&](double start, double stop, double step) {
                    std::vector<double> out;
                    if (step <= 0.0 || start > stop) {
                        out.push_back(start);
                        return out;
                    }
                    for (double value = start; value <= stop + 1e-9; value += step)
                        out.push_back(value);
                    if (out.empty())
                        out.push_back(start);
                    return out;
                };

                auto angles = make_float_range(range_start, range_stop, range_step);
                auto kps = make_double_range(range_kp_start, range_kp_stop, range_kp_step);
                auto kds = make_double_range(range_kd_start, range_kd_stop, range_kd_step);
                auto velocities = make_float_range(range_velocity_start, range_velocity_stop, range_velocity_step);
                size_t count = std::max({angles.size(), kps.size(), kds.size(), velocities.size()});
                sequence_steps.clear();
                for (size_t i = 0; i < count; ++i) {
                    SequenceStep step;
                    step.angle_deg = i < angles.size() ? angles[i] : angles.back();
                    step.kp = i < kps.size() ? kps[i] : kps.back();
                    step.kd = i < kds.size() ? kds[i] : kds.back();
                    // Calculate torque using τ = m·g·r·sin(θ)
                    float angle_rad = step.angle_deg * 3.14159265f / 180.0f;
                    step.torque_ff = calc_mass * g_accel * calc_length * std::sin(angle_rad);
                    // Apply scalar multiplier if set (range_torque_start != 0)
                    if (range_torque_start != 0.0f) {
                        step.torque_ff *= range_torque_start;
                    }
                    step.velocity_deg_s = i < velocities.size() ? velocities[i] : velocities.back();
                    sequence_steps.push_back(step);
                }
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::SeparatorText("Manual Step Entry");

            ImGui::SetNextItemWidth(110);
            ImGui::InputFloat("Angle (deg)##new_step", &new_sequence_step.angle_deg, 1.0f, 10.0f, "%.1f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kp##new_step", &new_sequence_step.kp, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(95);
            ImGui::InputDouble("Kd##new_step", &new_sequence_step.kd, 0.1, 1.0, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(105);
            ImGui::InputFloat("Torque##new_step", &new_sequence_step.torque_ff, 0.1f, 1.0f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::InputFloat("Velocity (deg/s)##new_step", &new_sequence_step.velocity_deg_s, 1.0f, 10.0f, "%.1f");
            ImGui::SameLine();
            if (ImGui::Button("Add Step##s", {90, 0}))
                sequence_steps.push_back(new_sequence_step);

            if (sequence_steps.empty()) {
                ImGui::TextDisabled("No sequence steps added yet.");
            } else if (ImGui::BeginTable("##sequence_steps", 7,
                       ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                       ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("Step", ImGuiTableColumnFlags_WidthFixed, 50);
                ImGui::TableSetupColumn("Angle (deg)", ImGuiTableColumnFlags_WidthFixed, 115);
                ImGui::TableSetupColumn("Kp", ImGuiTableColumnFlags_WidthFixed, 95);
                ImGui::TableSetupColumn("Kd", ImGuiTableColumnFlags_WidthFixed, 95);
                ImGui::TableSetupColumn("Torque", ImGuiTableColumnFlags_WidthFixed, 95);
                ImGui::TableSetupColumn("Velocity", ImGuiTableColumnFlags_WidthFixed, 110);
                ImGui::TableSetupColumn("Remove", ImGuiTableColumnFlags_WidthFixed, 70);
                ImGui::TableHeadersRow();

                for (int i = 0; i < (int)sequence_steps.size(); i++) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%d", i + 1);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::PushID(i);
                    ImGui::SetNextItemWidth(95);
                    ImGui::InputFloat("##angle", &sequence_steps[i].angle_deg, 1.0f, 10.0f, "%.1f");
                    ImGui::TableSetColumnIndex(2);
                    ImGui::SetNextItemWidth(80);
                    ImGui::InputDouble("##kp", &sequence_steps[i].kp, 0.1, 1.0, "%.2f");
                    ImGui::TableSetColumnIndex(3);
                    ImGui::SetNextItemWidth(80);
                    ImGui::InputDouble("##kd", &sequence_steps[i].kd, 0.1, 1.0, "%.2f");
                    ImGui::TableSetColumnIndex(4);
                    // Display calculated torque: τ = m·g·r·sin(θ)
                    float step_angle_rad = sequence_steps[i].angle_deg * 3.14159265f / 180.0f;
                    float calculated_step_torque = calc_mass * g_accel * calc_length * std::sin(step_angle_rad);
                    ImGui::Text("%.2f", calculated_step_torque);
                    ImGui::TableSetColumnIndex(5);
                    ImGui::SetNextItemWidth(95);
                    ImGui::InputFloat("##velocity", &sequence_steps[i].velocity_deg_s, 1.0f, 10.0f, "%.1f");
                    ImGui::TableSetColumnIndex(6);
                    if (ImGui::SmallButton("X")) {
                        sequence_steps.erase(sequence_steps.begin() + i);
                        ImGui::PopID();
                        break;
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        if (motors.empty()) {
            ImGui::TextDisabled("No motors added yet.");
        } else {
            if (ImGui::BeginTable("##motors", 6,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                    ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("Motor ID", ImGuiTableColumnFlags_WidthFixed, 70);
                ImGui::TableSetupColumn("Type",     ImGuiTableColumnFlags_WidthFixed, 70);
                ImGui::TableSetupColumn("Torque (Nm)", ImGuiTableColumnFlags_WidthFixed, 100);
                ImGui::TableSetupColumn("kp (Nm/rad)", ImGuiTableColumnFlags_WidthFixed, 110);
                ImGui::TableSetupColumn("kd (Nm/rad)", ImGuiTableColumnFlags_WidthFixed, 105);
                ImGui::TableSetupColumn("Remove",   ImGuiTableColumnFlags_WidthFixed, 60);
                ImGui::TableHeadersRow();

                for (int i = 0; i < (int)motors.size(); i++) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%d", motors[i].id);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%s", TYPE_NAMES[motors[i].type]);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::PushID(i);
                    ImGui::SetNextItemWidth(90);
                    ImGui::InputFloat("##torque", &motors[i].torque_ff, 0.1f, 1.0f, "%.2f");
                    ImGui::PopID();
                    ImGui::TableSetColumnIndex(3);
                    ImGui::PushID(i);
                    ImGui::SetNextItemWidth(100);
                    ImGui::InputDouble("##kp", &motors[i].kp, 0.1, 1.0, "%.2f");
                    ImGui::PopID();
                    ImGui::TableSetColumnIndex(4);
                    ImGui::PushID(i);
                    ImGui::SetNextItemWidth(90);
                    ImGui::InputDouble("##kd", &motors[i].kd, 0.1, 1.0, "%.2f");
                    ImGui::PopID();
                    ImGui::TableSetColumnIndex(5);
                    ImGui::PushID(i);
                    if (ImGui::SmallButton("X")) {
                        motors.erase(motors.begin() + i);
                        ImGui::PopID();
                        break;
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
        }

        // ---- add motor ---------------------------------------------------
        ImGui::Spacing();
        ImGui::SeparatorText("Add Motor");

        ImGui::SetNextItemWidth(100);
        ImGui::InputInt("Motor ID##new", &new_id);
        new_id = std::max(1, std::min(255, new_id));

        ImGui::SetNextItemWidth(120);
        ImGui::Combo("Motor Type##new", &new_type, TYPE_NAMES, TYPE_COUNT);

        if (ImGui::Button("Add Motor", {100, 30})) {
            MotorEntry e;
            e.id   = new_id;
            e.type = new_type;
            e.kp   = kp;    // inherit global kp
            e.kd   = kd;    // inherit global kd
            motors.push_back(e);
            new_id++;   // auto-increment for convenience
        }

        // ---- command preview ---------------------------------------------
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::SeparatorText("Command Preview");
        std::string overrun = overrun_opts[overrun_idx];
        std::string can_interface = build_can_interface_string(can_ifaces, can_iface_selected, 5);
        std::string preview = build_command(control_frequency_hz, cmd, target_deg, torque_val, zero_one_id, motors,
                            sequence_steps, hold_ms, sequence_cycles,
                            run_r_sec, run_w_sec, run_t_sec, overrun, can_interface);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
        ImGui::TextWrapped("%s", preview.c_str());
        ImGui::PopStyleColor();

        // ---- run / quit --------------------------------------------------
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::SeparatorText("Execute");

        bool can_run = !motors.empty() && !running_cmd.load() &&
                       (cmd != 5 || !sequence_steps.empty()) &&
                       !can_interface.empty();
        if (!can_run) ImGui::BeginDisabled();
        if (ImGui::Button(running_cmd.load() ? "Running..." : "Run Command", {150, 40})) {
            std::cout << preview << std::endl;
            std::string cmd_str = preview;
            running_cmd.store(true);
            std::thread([cmd_str, &running_cmd]() {
                int rc = system(cmd_str.c_str());
                if (rc != 0)
                    std::cerr << "Command failed with status " << rc << "\n";
                running_cmd.store(false);
            }).detach();
        }
        if (!can_run) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Quit", {100, 40})) running = false;

        ImGui::End();

        // ---- render ------------------------------------------------------
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(0.12f, 0.12f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
