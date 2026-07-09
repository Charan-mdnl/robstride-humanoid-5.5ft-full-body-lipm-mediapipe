// ik_mujoco_viewer.cpp
// MuJoCo physics viewer with two-thread architecture (physics + render)
// Subscribes to /joint_states and applies commands to actuators

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>

// IK markers from nr_ik_test (pelvis frame)
struct IKMarker {
    double x, y, z;
    float r, g, b, a;
    float size;
};

// Shared state
std::mutex state_mutex;
std::map<std::string, double> joint_positions;
std::vector<IKMarker> ik_markers;
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window = nullptr;

// Body IDs
int pelvis_id = -1;
int foot_r_id = -1;
int foot_l_id = -1;
std::map<std::string, int> actuator_map;

// Balance control gains (tunable via ROS params)
double balance_hip_gain = -0.4;
double balance_knee_gain = -0.4;
double balance_ankle_pitch_gain = -0.1;
double balance_ankle_roll_gain = -0.2;
std::mutex balance_gains_mutex;

// Viewer control
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    glfwGetCursorPos(window, &lastx, &lasty);
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right)
        return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

class RobotStateSubscriber : public rclcpp::Node {
public:
    RobotStateSubscriber() : Node("ik_mujoco_viewer") {
        // Declare tunable balance control parameters
        this->declare_parameter("balance_hip_gain", -0.4);
        this->declare_parameter("balance_knee_gain", -0.4);
        this->declare_parameter("balance_ankle_pitch_gain", -0.1);
        this->declare_parameter("balance_ankle_roll_gain", -0.2);

        // Read initial values
        update_balance_gains();

        // Create timer to periodically check for parameter updates
        param_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotStateSubscriber::update_balance_gains, this));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotStateSubscriber::joint_state_callback, this, std::placeholders::_1));

        marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/nr_ik_test/markers", 10,
            std::bind(&RobotStateSubscriber::marker_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MuJoCo viewer subscribed to /joint_states and /nr_ik_test/markers");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_positions[msg->name[i]] = msg->position[i];
        }
    }

    void marker_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        ik_markers.clear();
        for (const auto& mk : msg->markers) {
            IKMarker m;
            m.x = mk.pose.position.x;
            m.y = mk.pose.position.y;
            m.z = mk.pose.position.z;
            m.r = mk.color.r;
            m.g = mk.color.g;
            m.b = mk.color.b;
            m.a = mk.color.a;
            m.size = static_cast<float>(mk.scale.x) * 0.5f;  // sphere radius = half diameter
            ik_markers.push_back(m);
        }
    }

    void update_balance_gains() {
        std::lock_guard<std::mutex> lock(balance_gains_mutex);
        balance_hip_gain = this->get_parameter("balance_hip_gain").as_double();
        balance_knee_gain = this->get_parameter("balance_knee_gain").as_double();
        balance_ankle_pitch_gain = this->get_parameter("balance_ankle_pitch_gain").as_double();
        balance_ankle_roll_gain = this->get_parameter("balance_ankle_roll_gain").as_double();
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::TimerBase::SharedPtr param_timer_;
};

mjModel* load_xml_with_meshdir(const std::string& xml_path, const std::string& mesh_dir) {
    std::ifstream file(xml_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open XML file: " + xml_path);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string xml_content = buffer.str();
    
    // Replace MESHDIR_PLACEHOLDER with actual path
    size_t pos = xml_content.find("MESHDIR_PLACEHOLDER");
    if (pos != std::string::npos) {
        xml_content.replace(pos, 19, mesh_dir);
    }
    
    // Write to temp file (MuJoCo 3 loads from files, not strings)
    std::string temp_path = "/tmp/mujoco_temp.xml";
    std::ofstream temp_file(temp_path);
    if (!temp_file.is_open()) {
        throw std::runtime_error("Cannot create temporary file");
    }
    temp_file << xml_content;
    temp_file.close();
    
    // Load model from temp file
    char error[1000];
    mjModel* model = mj_loadXML(temp_path.c_str(), nullptr, error, 1000);
    if (!model) {
        throw std::runtime_error(std::string("Error loading model: ") + error);
    }
    
    return model;
}

void physics_thread() {
    // Standing posture defaults
    std::map<std::string, double> standing_targets;
    standing_targets["hip_pitch"] = -0.0873;      // -5 degrees
    standing_targets["knee_pitch"] = -0.2618;     // -15 degrees
    standing_targets["ankle_pitch"] = 0.1745;     // +10 degrees
    standing_targets["hip_roll"] = 0.0;
    standing_targets["ankle_roll"] = 0.0;
    standing_targets["thigh_yaw"] = 0.0;

    // PD control gains
    const double kp = 100.0;
    const double kd = 2.0;
    const double torque_limit = 120.0;

    while (!glfwWindowShouldClose(window)) {
        auto start = std::chrono::high_resolution_clock::now();

        state_mutex.lock();

        // Compute pelvis tilt and roll for balance feedback
        const mjtNum* quat = d->qpos + 3;  // pelvis quaternion [w, x, y, z]
        double tilt = 2.0 * (quat[0] * quat[2] - quat[3] * quat[1]);  // forward tilt (pitch)
        double roll = 2.0 * (quat[0] * quat[1] + quat[3] * quat[2]);  // sideways tilt (roll)
        double tilt_rate = d->qvel[4];  // angular velocity around pitch axis
        double roll_rate = d->qvel[3];  // angular velocity around roll axis
        double balance_pitch = -1.0 * tilt - 0.3 * tilt_rate;  // pitch balance correction
        double balance_roll = -1.0 * roll - 0.3 * roll_rate;   // roll balance correction

        // Position control: follow IK targets with light balance correction
        for (int i = 0; i < m->nu; i++) {
            int joint_id = m->actuator_trnid[i * 2];
            int q_idx = m->jnt_qposadr[joint_id];

            const char* joint_name = mj_id2name(m, mjOBJ_JOINT, joint_id);
            std::string name_str(joint_name ? joint_name : "");

            // Get base target: either from IK commands or standing default
            double target = 0.0;
            bool found_ik = false;
            for (const auto& [ik_name, ik_angle] : joint_positions) {
                if (name_str.find(ik_name) != std::string::npos) {
                    target = ik_angle;
                    found_ik = true;
                    break;
                }
            }
            if (!found_ik) {
                for (const auto& [key, val] : standing_targets) {
                    if (name_str.find(key) != std::string::npos) {
                        target = val;
                        break;
                    }
                }
            }

            // Apply balance corrections
            std::lock_guard<std::mutex> gains_lock(balance_gains_mutex);

            if (name_str.find("hip_pitch") != std::string::npos) {
                target += balance_hip_gain * balance_pitch;
            } else if (name_str.find("knee_pitch") != std::string::npos) {
                target += balance_knee_gain * balance_pitch;
            } else if (name_str.find("ankle_pitch") != std::string::npos) {
                target += balance_ankle_pitch_gain * balance_pitch;
            } else if (name_str.find("ankle_roll") != std::string::npos) {
                target += balance_ankle_roll_gain * balance_roll;
            }

            // Position actuators expect position targets
            d->ctrl[i] = target;
        }

        // Run physics step
        mj_step(m, d);

        // ── Debug print every 500 ms ──────────────────────────────────────────
        static auto last_print = std::chrono::high_resolution_clock::now();
        auto now_t = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now_t - last_print).count() >= 500) {
            last_print = now_t;

            // Robot CoM (world frame) — subtree_com[0] = whole-robot CoM
            const mjtNum* com = d->subtree_com;
            double com_x = com[0], com_y = com[1], com_z = com[2];

            // Pelvis pitch — extracted from xmat (row-major 3×3)
            // pitch = asin(-R[2][0]) = asin(-xmat[6])
            double pitch_rad = 0.0;
            if (pelvis_id >= 0) {
                const mjtNum* pm = d->xmat + 9 * pelvis_id;
                pitch_rad = std::asin(std::max(-1.0, std::min(1.0, -pm[6])));
            }

            // Contact normal forces per foot
            double fz_r = 0.0, fz_l = 0.0;
            static int contact_print_count = 0;
            bool print_contacts = (contact_print_count++ % 4 == 0);  // every 2s
            for (int ci = 0; ci < d->ncon; ci++) {
                mjtNum cf[6];
                mj_contactForce(m, d, ci, cf);
                int b1 = m->geom_bodyid[d->contact[ci].geom1];
                int b2 = m->geom_bodyid[d->contact[ci].geom2];
                if (print_contacts) {
                    const char* n1 = mj_id2name(m, mjOBJ_BODY, b1);
                    const char* n2 = mj_id2name(m, mjOBJ_BODY, b2);
                    std::cout << "  contact[" << ci << "] "
                        << (n1?n1:"?") << " vs " << (n2?n2:"?")
                        << "  dist=" << d->contact[ci].dist
                        << "  f0=" << cf[0] << std::endl;
                }
                if (b1 == foot_r_id || b2 == foot_r_id) fz_r += cf[0];
                if (b1 == foot_l_id || b2 == foot_l_id) fz_l += cf[0];
            }

            // Foot world positions
            double fr_x = 0, fr_y = 0, fr_z = 0, fl_x = 0, fl_y = 0, fl_z = 0;
            if (foot_r_id >= 0) { fr_x = d->xpos[foot_r_id*3]; fr_y = d->xpos[foot_r_id*3+1]; fr_z = d->xpos[foot_r_id*3+2]; }
            if (foot_l_id >= 0) { fl_x = d->xpos[foot_l_id*3]; fl_y = d->xpos[foot_l_id*3+1]; fl_z = d->xpos[foot_l_id*3+2]; }
            double mid_x = (fr_x + fl_x) * 0.5;
            double mid_y = (fr_y + fl_y) * 0.5;

            std::cout << std::fixed << std::setprecision(3)
                << "[DBG] CoM=(" << com_x << " " << com_y << " " << com_z << ")"
                << "  feet_mid=(" << mid_x << " " << mid_y << ")"
                << "  CoM_offset=(" << (com_x-mid_x) << " " << (com_y-mid_y) << ")"
                << "  pelvis_pitch=" << (pitch_rad * 180.0 / M_PI) << "deg"
                << "  foot_z_R=" << fr_z << "  foot_z_L=" << fl_z
                << "  ncon=" << d->ncon
                << "  Fz_R=" << fz_r << "N  Fz_L=" << fz_l << "N"
                << std::endl;
        }

        state_mutex.unlock();

        // Sleep to maintain timestep
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        auto sleep_time = std::chrono::microseconds((int)(m->opt.timestep * 1e6)) - elapsed;
        if (sleep_time.count() > 0) {
            std::this_thread::sleep_for(sleep_time);
        }
    }
}

void render_thread() {
    while (!glfwWindowShouldClose(window)) {
        state_mutex.lock();
        
        // Get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        
        // Update scene
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

        // Inject IK markers (pelvis frame → world frame)
        if (pelvis_id >= 0 && !ik_markers.empty()) {
            const mjtNum* pp = d->xpos + 3 * pelvis_id;
            const mjtNum* pm = d->xmat + 9 * pelvis_id;
            for (const auto& mk : ik_markers) {
                if (scn.ngeom >= scn.maxgeom) break;
                // rotate and translate into world frame
                mjtNum pw[3];
                pw[0] = pp[0] + pm[0]*mk.x + pm[1]*mk.y + pm[2]*mk.z;
                pw[1] = pp[1] + pm[3]*mk.x + pm[4]*mk.y + pm[5]*mk.z;
                pw[2] = pp[2] + pm[6]*mk.x + pm[7]*mk.y + pm[8]*mk.z;
                mjtNum sz[3]  = {mk.size, mk.size, mk.size};
                mjtNum mat[9] = {1,0,0, 0,1,0, 0,0,1};
                float  rgba[4] = {mk.r, mk.g, mk.b, mk.a};
                mjv_initGeom(scn.geoms + scn.ngeom, mjGEOM_SPHERE, sz, pw, mat, rgba);
                scn.ngeom++;
            }
        }

        // Render
        mjr_render(viewport, &scn, &con);
        
        // Swap buffers
        glfwSwapBuffers(window);
        
        state_mutex.unlock();
        
        // Poll events
        glfwPollEvents();
        
        // 50 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

int main(int argc, char** argv) {
    // Load MuJoCo plugins (for STL mesh loading etc.)
    const char* plugin_dir = std::getenv("MUJOCO_PLUGIN_PATH");
    if (plugin_dir) {
        std::cout << "[ik_mujoco_viewer] Loading plugins from: " << plugin_dir << std::endl;
        mj_loadAllPluginLibraries(plugin_dir, +[](const char* filename, int first, int count) {
            std::cout << "  Loaded " << count << " plugins from " << filename << std::endl;
        });
    }
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStateSubscriber>();
    
    // Load MuJoCo model
    const std::string xml_path = "/home/me/clean_ws/install/full_body_mujoco/share/full_body_mujoco/config/angad_full_body.xml";
    const std::string mesh_dir = "/home/me/clean_ws/src/XP_robot_with_hand_mjcf/meshes";
    
    std::cout << "[ik_mujoco_viewer] Loading model..." << std::endl;
    try {
        m = load_xml_with_meshdir(xml_path, mesh_dir);
    } catch (const std::exception& e) {
        std::cerr << "Load model error: " << e.what() << std::endl;
        return 1;
    }
    
    d = mj_makeData(m);
    
    // Reset to standing keyframe
    int keyframe_id = mj_name2id(m, mjOBJ_KEY, "standing");
    if (keyframe_id >= 0) {
        mj_resetDataKeyframe(m, d, keyframe_id);
    }
    
    // Build actuator map and initialize
    for (int i = 0; i < m->nu; i++) {
        std::string act_name = mj_id2name(m, mjOBJ_ACTUATOR, i);
        actuator_map[act_name] = i;
        
        // Initialize actuator to keyframe position
        int jnt_id = m->actuator_trnid[i*2];
        if (jnt_id >= 0 && jnt_id < m->njnt) {
            int qpos_adr = m->jnt_qposadr[jnt_id];
            d->ctrl[i] = d->qpos[qpos_adr];
        }
    }
    
    // Print body masses and inertias for inspection
    std::cout << "\n[ik_mujoco_viewer] Body inertias (name  mass  Ixx Iyy Izz):\n";
    for (int i = 1; i < m->nbody; i++) {  // skip world body (0)
        const char* name = mj_id2name(m, mjOBJ_BODY, i);
        double mass = m->body_mass[i];
        double* I = m->body_inertia + 3 * i;
        std::cout << "  " << (name ? name : "?")
                  << "  m=" << mass
                  << "  I=(" << I[0] << " " << I[1] << " " << I[2] << ")\n";
    }
    std::cout << std::endl;

    // Scale contact force arrows so they are readable (default is very large)
    m->vis.map.force = 0.002f;  // scale contact force arrow length

    // Get body IDs
    pelvis_id = mj_name2id(m, mjOBJ_BODY, "pelvis");
    foot_r_id  = mj_name2id(m, mjOBJ_BODY, "foot_r");
    foot_l_id  = mj_name2id(m, mjOBJ_BODY, "foot_l");
    
    std::cout << "[ik_mujoco_viewer] Model loaded: " << m->nq << " DoF, " 
              << m->nu << " actuators" << std::endl;
    std::cout << "[ik_mujoco_viewer] Physics: " << (int)(1.0/m->opt.timestep) << " Hz" << std::endl;
    
    // Initialize GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
        return 1;
    }
    
    // Create window
    window = glfwCreateWindow(1200, 900, "Angad Full Body - MuJoCo Viewer", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    // Initialize visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    
    // Set camera
    cam.distance = 2.5;
    cam.elevation = -20;
    cam.azimuth = 120;
    
    // Install callbacks
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, [](GLFWwindow*, int key, int, int act, int) {
        if (act != GLFW_PRESS) return;
        if (key == GLFW_KEY_I)
            opt.flags[mjVIS_INERTIA] ^= 1;
        if (key == GLFW_KEY_C)
            opt.flags[mjVIS_COM] ^= 1;
        if (key == GLFW_KEY_F)
            opt.flags[mjVIS_CONTACTFORCE] ^= 1;  // toggle contact forces
    });
    
    std::cout << "[ik_mujoco_viewer] Starting physics and render threads..." << std::endl;
    
    // Start ROS2 thread (now that window is created)
    std::thread ros_thread([node]() {
        while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    
    // Start physics thread (keep joinable for clean shutdown)
    std::thread phys_thread(physics_thread);
    
    // Run render in main thread
    render_thread();
    
    // Wait for physics thread to finish
    if (phys_thread.joinable()) {
        phys_thread.join();
    }
    
    // Cleanup - shutdown ROS and wait for thread to finish
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    
    glfwTerminate();
    
    return 0;
}
