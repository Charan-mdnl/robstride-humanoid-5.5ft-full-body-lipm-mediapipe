// DCM-based Walking Controller with LIPM Stabilization
// Integrates architecture from lipm_control library:
// - DCM (Divergent Component of Motion) feedback control
// - CoM admittance control with leaky integrators  
// - Numerical IK (Levenberg-Marquardt from Kajita)
// - Force sensor and IMU feedback

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker_array.hpp>

// ═══════════════════════════════════════════════════════════════════════════
// Leaky Integrator (from lipm_control/LeakyIntegrator.h)
// ═══════════════════════════════════════════════════════════════════════════
class LeakyIntegrator {
  Eigen::Vector3d sum_{Eigen::Vector3d::Zero()};
  double dt_{0.02};
  double leak_{0.95};  // Stronger leak to prevent unbounded growth
public:
  void add(const Eigen::Vector3d &v, double dt) {
    sum_ = leak_ * sum_ + v * dt;
    dt_ = dt;
  }
  Eigen::Vector3d eval() const { return sum_; }
  void reset() { sum_.setZero(); }
};

// ═══════════════════════════════════════════════════════════════════════════
// DCM Controller (from lipm_control/LIPMControl.h)
// ═══════════════════════════════════════════════════════════════════════════
class DCMController {
  double omega_{0};           // sqrt(g/h)
  double dt_{0.02};
  Eigen::Matrix3d Kzmp_, Kdcm_;
  
  bool initialized_{false};
  Eigen::Vector3d CoM_cmd_, dCoM_cmd_, ddCoM_cmd_;

public:
  DCMController(double h, double dt,
                double Pdcm_x = 0.3, double Pdcm_y = 0.3)
      : dt_(dt) {
    omega_ = std::sqrt(9.81 / h);
    
    Kzmp_.setZero();
    Kzmp_(0, 0) = 0.0;  // No ZMP feedback
    Kzmp_(1, 1) = 0.0;
    
    Kdcm_.setZero();
    Kdcm_(0, 0) = Pdcm_x;
    Kdcm_(1, 1) = Pdcm_y;
    
    CoM_cmd_.setZero();
    dCoM_cmd_.setZero();
    ddCoM_cmd_.setZero();
  }
  
  void setOmega(double omega) { omega_ = omega; }
  double getOmega() const { return omega_; }
  
  Eigen::Vector3d computeDCM(const Eigen::Vector3d &CoM,
                              const Eigen::Vector3d &dCoM) const {
    return CoM + (1.0 / omega_) * dCoM;
  }
  
  // Main control loop (§ lipm_control/LIPMControl.h Control())
  void control(const Eigen::Vector3d &ZMP_meas,
               const Eigen::Vector3d &CoM_meas,
               const Eigen::Vector3d &dCoM_meas,
               const Eigen::Vector3d &CoM_des,
               const Eigen::Vector3d &dCoM_des,
               const Eigen::Vector3d &ddCoM_des) {
    if (!initialized_) {
      CoM_cmd_ = CoM_des;
      dCoM_cmd_ = dCoM_des;
      ddCoM_cmd_ = ddCoM_des;
      initialized_ = true;
      return;
    }
    
    // Compute DCM states
    Eigen::Vector3d DCM_ref = computeDCM(CoM_des, dCoM_des);
    Eigen::Vector3d DCM_meas = computeDCM(CoM_meas, dCoM_meas);
    
    // Simple proportional DCM tracking: CoM_cmd = CoM_des + K_dcm * (DCM_ref - DCM_meas)
    Eigen::Vector3d DCM_error = DCM_ref - DCM_meas;
    Eigen::Vector3d CoM_correction = Kdcm_ * DCM_error;
    
    CoM_cmd_ = CoM_des + CoM_correction;
    dCoM_cmd_ = dCoM_des;
    
    // Store for queries
    ddCoM_cmd_ = ddCoM_des;
  }
  
  Eigen::Vector3d getCoMCmd() const { return CoM_cmd_; }
  Eigen::Vector3d getDCoMCmd() const { return dCoM_cmd_; }
  Eigen::Vector3d getDDCoMCmd() const { return ddCoM_cmd_; }
  
  void reset() {
    initialized_ = false;
  }
};

// ═══════════════════════════════════════════════════════════════════════════
// Data Structures
// ═══════════════════════════════════════════════════════════════════════════
struct TrajPoint {
  double t, x, xdot, y, ydot;
};
struct Footstep {
  double x, y;
};
struct LegAngles {
  double hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll;
};
struct ForceSensor {
  bool is_right;
  double x, y, fz;
};

// ═══════════════════════════════════════════════════════════════════════════
// Main Walker Node
// ═══════════════════════════════════════════════════════════════════════════
class DCMWalker : public rclcpp::Node {
public:
  DCMWalker()
      : Node("dcm_walker"), qos_(rclcpp::QoS(1).transient_local()),
        startup_(now()) {
    // DCM control parameters (tunable at runtime)
    declare_parameter("Pdcm_x", 0.3);
    declare_parameter("Pdcm_y", 0.3);
    
    // Walking parameters
    declare_parameter("step_length", 0.08);
    declare_parameter("step_height", 0.04);
    declare_parameter("foot_separation", 0.08);
    declare_parameter("t_ss", 0.8);
    declare_parameter("t_ds", 0.5);
    declare_parameter("num_steps", 6);
    
    // Feedback gains (optional)
    declare_parameter("kp_pitch", 0.0);
    declare_parameter("kd_pitch", 0.0);
    declare_parameter("kp_roll", 0.0);
    declare_parameter("kd_roll", 0.0);
    declare_parameter("max_corr", 0.15);
    
    declare_parameter("reset", false);
    
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          for (auto &p : params) {
            if (p.get_name() == "reset" && p.as_bool()) {
              resetWalk();
            }
          }
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = true;
          return r;
        });
    
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description", qos_,
        [this](std_msgs::msg::String::SharedPtr m) { onUrdf(m); });
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu_site_imu_sensor/imu", 10,
        [this](sensor_msgs::msg::Imu::SharedPtr m) { onImu(m); });
    
    // Force sensors (8 total, 4 per foot)
    setupForceSensors();
    
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, this);
    walk_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/lower_body_controller/joint_trajectory", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "walk_markers", 10);
    
    com_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                    [this]() { onComTimer(); });
    ctrl_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                     [this]() { onControl(); });
    
    RCLCPP_INFO(get_logger(), "DCMWalker started with LIPM stabilization");
  }

private:
  // ───────────────────────────────────────────────────────────────────────
  // Force Sensor Utilities
  // ───────────────────────────────────────────────────────────────────────
  void setupForceSensors() {
    fsens_[0] = {true, 0.0, 0.075, 0};
    fsens_[1] = {true, 0.015, 0.075, 0};
    fsens_[2] = {true, 0.0, -0.048, 0};
    fsens_[3] = {true, -0.015, -0.048, 0};
    fsens_[4] = {false, 0.0, 0.075, 0};
    fsens_[5] = {false, -0.015, 0.075, 0};
    fsens_[6] = {false, 0.0, -0.048, 0};
    fsens_[7] = {false, 0.015, -0.048, 0};
    
    const char *rt[] = {"/foot_r_toe_site_wrench_sensor/wrench",
                        "/foot_r_toe_site_2_wrench_sensor/wrench",
                        "/foot_r_heel_site_wrench_sensor/wrench",
                        "/foot_r_heel_site_2_wrench_sensor/wrench"};
    const char *lt[] = {"/foot_l_toe_site_wrench_sensor/wrench",
                        "/foot_l_toe_site_2_wrench_sensor/wrench",
                        "/foot_l_heel_site_wrench_sensor/wrench",
                        "/foot_l_heel_site_2_wrench_sensor/wrench"};
    
    for (int i = 0; i < 4; ++i) {
      fsub_[i] = create_subscription<geometry_msgs::msg::WrenchStamped>(
          rt[i], rclcpp::SensorDataQoS(),
          [this, i](geometry_msgs::msg::WrenchStamped::SharedPtr m) {
            fsens_[i].fz = m->wrench.force.z;
          });
      fsub_[i + 4] = create_subscription<geometry_msgs::msg::WrenchStamped>(
          lt[i], rclcpp::SensorDataQoS(),
          [this, i](geometry_msgs::msg::WrenchStamped::SharedPtr m) {
            fsens_[i + 4].fz = m->wrench.force.z;
          });
    }
  }
  
  std::tuple<double, double, double> footCoP(bool right) const {
    double sf = 0, sfx = 0, sfy = 0;
    for (int i = 0; i < 8; ++i) {
      if (fsens_[i].is_right != right)
        continue;
      double f = std::abs(fsens_[i].fz);
      sf += f;
      sfx += f * fsens_[i].x;
      sfy += f * fsens_[i].y;
    }
    if (sf < 1.0)
      return {0, 0, 0};
    return {sf, sfx / sf, sfy / sf};
  }
  
  std::tuple<double, double, double> totalZMP() const {
    auto [rf, rx, ry] = footCoP(true);
    auto [lf, lx, ly] = footCoP(false);
    double t = rf + lf;
    if (t < 1)
      return {0, 0, 0};
    // ZMP in pelvis frame
    return {t, (rf * rx + lf * lx) / t, (rf * ry + lf * ly) / t};
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Numerical IK (Kajita-style Levenberg-Marquardt)
  // ───────────────────────────────────────────────────────────────────────
  static Eigen::Vector3d rot2omega(const Eigen::Matrix3d &R) {
    Eigen::Vector3d el(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0),
                       R(1, 0) - R(0, 1));
    double n = el.norm();
    if (n > 1e-10)
      return std::atan2(n, R.trace() - 1.0) / n * el;
    if (R(0, 0) > 0 && R(1, 1) > 0 && R(2, 2) > 0)
      return Eigen::Vector3d::Zero();
    return (M_PI / 2.0) * Eigen::Vector3d(R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1);
  }
  
  static Eigen::Matrix<double, 6, 1>
  calcVWerr(const Eigen::Vector3d &p_ref, const Eigen::Matrix3d &R_ref,
            const Eigen::Vector3d &p_now, const Eigen::Matrix3d &R_now) {
    Eigen::Matrix<double, 6, 1> err;
    err.head<3>() = p_ref - p_now;
    err.tail<3>() = R_now * rot2omega(R_now.transpose() * R_ref);
    return err;
  }
  
  struct LegFKFull {
    Eigen::Vector3d p_end;
    Eigen::Matrix3d R_end;
    Eigen::Vector3d joint_p[5];
    Eigen::Vector3d joint_a[5];
  };
  
  LegFKFull legFKFull(const Eigen::VectorXd &q, bool right) const {
    const double hx = -0.149, hy = right ? -0.122 : 0.122, hz = -0.103;
    const double L1 = 0.2517, L2 = 0.3472;
    const double afo_x = 0.017, afo_y = right ? -0.023 : 0.023, afo_z = -0.030;
    const double rs = right ? -1.0 : 1.0;
    
    LegFKFull res;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p(hx, hy, hz);
    
    auto joint = [&](int i, const Eigen::Vector3d &axis, double angle) {
      res.joint_p[i] = p;
      res.joint_a[i] = R * axis;
      R *= Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    };
    
    joint(0, Eigen::Vector3d::UnitX(), q(0));        // hip roll
    joint(1, Eigen::Vector3d::UnitY(), q(1));        // hip pitch
    p += R * Eigen::Vector3d(0.0, 0.0, -L1);
    joint(2, Eigen::Vector3d::UnitY(), q(2));        // knee pitch
    p += R * Eigen::Vector3d(0.0, 0.0, -L2);
    joint(3, Eigen::Vector3d::UnitY(), q(3));        // ankle pitch
    
    res.joint_p[4] = p;
    res.joint_a[4] = R * (rs * Eigen::Vector3d::UnitX());
    R *= Eigen::AngleAxisd(rs * q(4), Eigen::Vector3d::UnitX())
             .toRotationMatrix();
    
    p += R * Eigen::Vector3d(afo_x, afo_y, afo_z);
    res.p_end = p;
    res.R_end = R;
    return res;
  }
  
  static Eigen::Matrix<double, 6, 5> calcJacobian(const LegFKFull &fk) {
    Eigen::Matrix<double, 6, 5> J;
    for (int j = 0; j < 5; ++j) {
      J.block<3, 1>(0, j) = fk.joint_a[j].cross(fk.p_end - fk.joint_p[j]);
      J.block<3, 1>(3, j) = fk.joint_a[j];
    }
    return J;
  }
  
  LegAngles numericalLegIK(double fx, double fy, double fz, double tgt_pitch,
                            double tgt_roll, const LegAngles &guess,
                            bool right) const {
    const Eigen::Vector3d p_ref(fx, fy, fz);
    const Eigen::Matrix3d R_ref =
        Eigen::AngleAxisd(tgt_roll, Eigen::Vector3d::UnitX())
            .toRotationMatrix() *
        Eigen::AngleAxisd(tgt_pitch, Eigen::Vector3d::UnitY())
            .toRotationMatrix();
    
    // Error weights: high for position, very low for orientation (ignore roll)
    Eigen::Matrix<double, 6, 1> wv;
    wv << 1.0 / 0.3, 1.0 / 0.3, 1.0 / 0.3, 1.0 / (20 * M_PI), 1.0 / (20 * M_PI),
        1.0 / (20 * M_PI);
    
    Eigen::VectorXd q(5);
    q << guess.hip_roll, guess.hip_pitch, guess.knee_pitch, guess.ankle_pitch,
        right ? -guess.ankle_roll : guess.ankle_roll;
    
    LegFKFull fk = legFKFull(q, right);
    Eigen::Matrix<double, 6, 1> err =
        calcVWerr(p_ref, R_ref, fk.p_end, fk.R_end);
    double Ek = (wv.array() * err.array().square()).sum();
    
    for (int iter = 0; iter < 10; ++iter) {
      auto J = calcJacobian(fk);
      double lambda = Ek + 0.002;
      Eigen::Matrix<double, 5, 5> Hk =
          J.transpose() * (wv.asDiagonal() * J) +
          lambda * Eigen::Matrix<double, 5, 5>::Identity();
      Eigen::Matrix<double, 5, 1> gk = J.transpose() * (wv.asDiagonal() * err);
      Eigen::VectorXd dq = Hk.ldlt().solve(gk);
      
      q += dq;
      fk = legFKFull(q, right);
      err = calcVWerr(p_ref, R_ref, fk.p_end, fk.R_end);
      double Ek2 = (wv.array() * err.array().square()).sum();
      
      if (Ek2 < 1e-12)
        break;
      else if (Ek2 < Ek)
        Ek = Ek2;
      else {
        q -= dq;
        break;
      }
    }
    
    double ank_roll_cmd = 0.0;  // Ignore roll entirely
    return {0.0, q(0), q(1), q(2), q(3), ank_roll_cmd};
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Joint Configuration Utilities
  // ───────────────────────────────────────────────────────────────────────
  static auto jointNames() {
    return std::array<std::string, 12>{
        "hip_pitch_r",   "hip_roll_r",   "thigh_yaw_r",   "knee_pitch_r",
        "ankle_pitch_r", "ankle_roll_r", "hip_pitch_l",   "hip_roll_l",
        "thigh_yaw_l",   "knee_pitch_l", "ankle_pitch_l", "ankle_roll_l"};
  }
  
  static auto crouchAngles() {
    return std::array<double, 12>{-0.35, 0.0, 0.0, -0.45, -0.30, 0.0,
                                   -0.35, 0.0, 0.0, -0.45, -0.30, 0.0};
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Callbacks
  // ───────────────────────────────────────────────────────────────────────
  void onUrdf(std_msgs::msg::String::SharedPtr msg) {
    if (!model_.initString(msg->data))
      return;
    for (auto &[name, link] : model_.links_) {
      if (!link->inertial)
        continue;
      link_mass_[name] = link->inertial->mass;
      link_com_[name] = {link->inertial->origin.position.x,
                         link->inertial->origin.position.y,
                         link->inertial->origin.position.z};
      total_mass_ += link->inertial->mass;
    }
    RCLCPP_INFO(get_logger(), "URDF: %zu links, %.1f kg", link_mass_.size(),
                total_mass_);
    urdf_ok_ = true;
    urdf_sub_.reset();
  }
  
  void onImu(sensor_msgs::msg::Imu::SharedPtr m) {
    double az = m->linear_acceleration.z;
    double ax = m->linear_acceleration.x;
    double ay = m->linear_acceleration.y;
    double g = std::sqrt(ax * ax + ay * ay + az * az);
    if (g < 0.1)
      return;
    imu_pitch_ = std::asin(std::max(-1.0, std::min(1.0, -ay / g)));
    imu_roll_ = std::atan2(ax, az);
    imu_pitch_rate_ = m->angular_velocity.x;
    imu_roll_rate_ = m->angular_velocity.y;
    imu_ok_ = true;
  }
  
  void onComTimer() {
    if (!urdf_ok_)
      return;
    double wx = 0, wy = 0, wz = 0, mu = 0;
    for (auto &[name, mass] : link_mass_) {
      geometry_msgs::msg::PointStamped lp, wp;
      lp.header.frame_id = name;
      lp.point.x = link_com_[name][0];
      lp.point.y = link_com_[name][1];
      lp.point.z = link_com_[name][2];
      try {
        wp = tf_buf_->transform(lp, "pelvis", tf2::durationFromSec(0.05));
      } catch (...) {
        continue;
      }
      wx += mass * wp.point.x;
      wy += mass * wp.point.y;
      wz += mass * wp.point.z;
      mu += mass;
    }
    if (mu < 0.1)
      return;
    com_x_ = wx / mu;
    com_y_ = wy / mu;
    com_z_ = wz / mu;
    
    // Estimate CoM velocity via finite difference
    static double com_x_prev = com_x_, com_y_prev = com_y_;
    static auto t_prev = now();
    double dt = (now() - t_prev).seconds();
    if (dt > 0.01 && dt < 1.0) {
      dcom_x_ = (com_x_ - com_x_prev) / dt;
      dcom_y_ = (com_y_ - com_y_prev) / dt;
      com_x_prev = com_x_;
      com_y_prev = com_y_;
      t_prev = now();
    }
    
    try {
      geometry_msgs::msg::PointStamped fp, fp2;
      fp.header.frame_id = "foot_r";
      fp.point = {};
      fp2 = tf_buf_->transform(fp, "pelvis", tf2::durationFromSec(0.05));
      if (!walk_active_) {
        h_foot_ = fp2.point.z;
        double h = wz / mu - fp2.point.z;
        omega0_ = std::sqrt(9.81 / h);
        com_h_ = h;
        omega0_ok_ = true;
        
        // Initialize DCM controller with measured height
        if (!dcm_ctrl_) {
          double Pdcm_x = get_parameter("Pdcm_x").as_double();
          double Pdcm_y = get_parameter("Pdcm_y").as_double();
          dcm_ctrl_ = std::make_unique<DCMController>(h, 0.02, Pdcm_x, Pdcm_y);
          RCLCPP_INFO(get_logger(), "DCM Controller initialized: h=%.3f ω=%.3f",
                      h, omega0_);
        }
      }
    } catch (...) {
    }
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Main Control Loop
  // ───────────────────────────────────────────────────────────────────────
  void onControl() {
    double t = (now() - startup_).seconds();
    double settle_dur = 6.0;
    
    // Phase A: Crouch and settle
    if (!settle_ok_) {
      auto angs = crouchAngles();
      publishAngles(angs, 1.5);
      if (t > settle_dur) {
        settle_ok_ = true;
        RCLCPP_INFO(get_logger(), "Settle complete");
      }
      return;
    }
    
    if (!omega0_ok_ || !dcm_ctrl_)
      return;
    
    // Phase B: Initialize walk
    if (!walk_active_) {
      if (walk_done_)
        return;
      try {
        initFootsteps();
        initTrajectory();
        captureInitialFK();
        walk_active_ = true;
        walk_start_ = now();
        phase_start_ = now();
        phase_ = DS;
        step_ = 0;
        dcm_ctrl_->reset();
        RCLCPP_INFO(get_logger(), "Walk started with DCM control");
      } catch (tf2::TransformException &e) {
        RCLCPP_WARN(get_logger(), "Failed to init walk: %s", e.what());
        return;
      }
    }
    
    // Phase C: State machine transitions
    updateStateMachine();
    
    if (!walk_active_)
      return;
    
    // Phase D: Compute ZMP reference and trajectory tracking
    double alpha = computePhaseAlpha();
    auto [zmp_ref_x, zmp_ref_y] = computeZMPReference(alpha);
    auto [com_des_x, com_des_y, dcom_des_x, dcom_des_y, ddcom_des_x, ddcom_des_y] =
        getTrajectoryState();
    
    // Phase E: Measure current ZMP from force sensors
    auto [total_force, zmp_meas_x, zmp_meas_y] = totalZMP();
    
    // Phase F: DCM feedback control
    Eigen::Vector3d ZMP_meas(zmp_meas_x, zmp_meas_y, 0);
    Eigen::Vector3d CoM_meas(com_x_, com_y_, com_z_);
    Eigen::Vector3d dCoM_meas(dcom_x_, dcom_y_, 0);
    Eigen::Vector3d CoM_des(com_des_x, com_des_y, com_z_);
    Eigen::Vector3d dCoM_des(dcom_des_x, dcom_des_y, 0);
    Eigen::Vector3d ddCoM_des(ddcom_des_x, ddcom_des_y, 0);
    
    dcm_ctrl_->control(ZMP_meas, CoM_meas, dCoM_meas, CoM_des, dCoM_des,
                       ddCoM_des);
    
    // Get commanded CoM from DCM controller
    Eigen::Vector3d CoM_cmd = dcm_ctrl_->getCoMCmd();
    double cx = CoM_cmd(0);
    double cy = CoM_cmd(1);
    
    // Phase G: Compute foot targets and IK
    auto [r_fx, r_fy, r_fz, l_fx, l_fy, l_fz] = computeFootTargets(cx, cy, alpha);
    
    auto r_ik = numericalLegIK(r_fx, r_fy, r_fz, 0.0, 0.0, r_ik_last_, true);
    auto l_ik = numericalLegIK(l_fx, l_fy, l_fz, 0.0, 0.0, l_ik_last_, false);
    r_ik_last_ = r_ik;
    l_ik_last_ = l_ik;
    
    // Phase H: Convert to joint commands
    std::array<double, 12> j;
    legAnglesToJoints(r_ik, l_ik, j);
    
    // Phase I: Optional feedback (IMU, CoP)
    applyFeedback(j, total_force);
    
    // Phase J: Publish and log
    publishAngles(j, 0.08);
    publishMarkers(cx, cy, zmp_ref_x, zmp_ref_y);
    logState(alpha, cx, cy, r_fx, r_fy, r_fz, l_fx, l_fy, l_fz, r_ik, l_ik);
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // State Machine
  // ───────────────────────────────────────────────────────────────────────
  void updateStateMachine() {
    double pt = (now() - phase_start_).seconds();
    double tss = get_parameter("t_ss").as_double();
    double tds = get_parameter("t_ds").as_double();
    
    if (phase_ == DS) {
      if (pt > tds && step_ + 1 < N_) {
        step_++;
        phase_ = (step_ % 2 == 0) ? SR : SL;
        phase_start_ = now();
      }
    } else {
      if (pt > tss) {
        if (step_ >= N_ - 2) {
          walk_active_ = false;
          walk_done_ = true;
          RCLCPP_INFO(get_logger(), "Walk finished");
          return;
        }
        phase_ = DS;
        phase_start_ = now();
      }
    }
  }
  
  double computePhaseAlpha() const {
    double pt = (now() - phase_start_).seconds();
    double tss = get_parameter("t_ss").as_double();
    double tds = get_parameter("t_ds").as_double();
    if (phase_ == DS)
      return std::min(pt / tds, 1.0);
    else
      return std::min(pt / tss, 1.0);
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Trajectory Generation & Tracking
  // ───────────────────────────────────────────────────────────────────────
  void initFootsteps() {
    auto fr = tf_buf_->lookupTransform("pelvis", "foot_r", tf2::TimePointZero);
    auto fl = tf_buf_->lookupTransform("pelvis", "foot_l", tf2::TimePointZero);
    
    double fr_x = fr.transform.translation.x;
    double fr_y = fr.transform.translation.y;
    double fl_x = fl.transform.translation.x;
    double fl_y = fl.transform.translation.y;
    
    walk_origin_x_ = (fr_x + fl_x) * 0.5;
    walk_origin_y_ = (fr_y + fl_y) * 0.5;
    
    double sep = get_parameter("foot_separation").as_double();
    foot_r_y_ = -sep;
    foot_l_y_ = sep;
    
    double sl = get_parameter("step_length").as_double();
    int N = get_parameter("num_steps").as_int();
    N_ = N;
    
    steps_.clear();
    for (int i = 0; i < N_; i++) {
      double x = (i <= 1) ? 0.0 : (i - 1) * sl;
      steps_.push_back({x, (i % 2 == 0) ? foot_r_y_ : foot_l_y_});
    }
    RCLCPP_INFO(get_logger(), "Footsteps: origin=(%.3f, %.3f) N=%d",
                walk_origin_x_, walk_origin_y_, N_);
  }
  
  void initTrajectory() {
    double tss = get_parameter("t_ss").as_double();
    double sl = get_parameter("step_length").as_double();
    
    double wT = omega0_ * tss;
    double xdot0 = sl * omega0_ * sinh(wT) / (2 * (cosh(wT) - 1));
    double x_rel = xdot0 * (1 - cosh(wT)) / (omega0_ * sinh(wT));
    double y0 = (steps_[0].y + steps_[1].y) * 0.5;
    double yoff = steps_[1].y - y0;
    double ydot0 = yoff * omega0_ * sinh(wT) / (cosh(wT) + 1);
    
    traj_.clear();
    double x = steps_[0].x + x_rel, y = y0;
    double xdot = xdot0, ydot = ydot0;
    
    for (int i = 0; i + 1 < N_; i++) {
      double zx = steps_[i + 1].x, zy = steps_[i + 1].y;
      for (double t = 0; t < tss - 1e-9; t += 0.02) {
        double c = cosh(omega0_ * t);
        double s = sinh(omega0_ * t);
        double xn = (x - zx) * c + (xdot / omega0_) * s + zx;
        double xdn = (x - zx) * omega0_ * s + xdot * c;
        double yn = (y - zy) * c + (ydot / omega0_) * s + zy;
        double ydn = (y - zy) * omega0_ * s + ydot * c;
        traj_.push_back({i * tss + t, xn, xdn, yn, ydn});
      }
      x = steps_[i + 1].x + x_rel;
      xdot = xdot0;
      y = (i % 2 == 0) ? (y0 + yoff) : (y0 - yoff);
      ydot = (i % 2 == 0) ? -ydot0 : ydot0;
    }
    RCLCPP_INFO(get_logger(), "Trajectory: %zu points", traj_.size());
  }
  
  std::tuple<double, double, double, double, double, double>
  getTrajectoryState() const {
    if (traj_.empty())
      return {0, 0, 0, 0, 0, 0};
    
    double t_walk = (now() - walk_start_).seconds();
    size_t idx = std::min(size_t(t_walk / 0.02), traj_.size() - 1);
    
    auto &tp = traj_[idx];
    double ddx = omega0_ * omega0_ * (tp.x - steps_[step_].x);
    double ddy = omega0_ * omega0_ * (tp.y - steps_[step_].y);
    
    // Transform: robot_x = walk_y, robot_y = -walk_x
    return {tp.y, -tp.x, tp.ydot, -tp.xdot, ddy, -ddx};
  }
  
  std::tuple<double, double> computeZMPReference(double alpha) const {
    if (phase_ == DS && step_ + 1 < N_) {
      double zx = steps_[step_].x + alpha * (steps_[step_ + 1].x - steps_[step_].x);
      double zy = steps_[step_].y + alpha * (steps_[step_ + 1].y - steps_[step_].y);
      // Transform: robot_x = walk_y, robot_y = -walk_x
      return {zy, -zx};
    }
    // Transform: robot_x = walk_y, robot_y = -walk_x
    return {steps_[step_].y, -steps_[step_].x};
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Foot Placement
  // ───────────────────────────────────────────────────────────────────────
  void captureInitialFK() {
    try {
      geometry_msgs::msg::PointStamped orig, pt;
      orig.point.x = orig.point.y = orig.point.z = 0;
      orig.header.frame_id = "foot_r";
      pt = tf_buf_->transform(orig, "pelvis", tf2::durationFromSec(0.05));
      init_r_x_ = pt.point.x;
      init_r_y_ = pt.point.y;
      init_r_z_ = pt.point.z;
      
      orig.header.frame_id = "foot_l";
      pt = tf_buf_->transform(orig, "pelvis", tf2::durationFromSec(0.05));
      init_l_x_ = pt.point.x;
      init_l_y_ = pt.point.y;
      init_l_z_ = pt.point.z;
      
      RCLCPP_INFO(get_logger(),
                  "Initial FK: R=(%.3f,%.3f,%.3f) L=(%.3f,%.3f,%.3f)",
                  init_r_x_, init_r_y_, init_r_z_, init_l_x_, init_l_y_,
                  init_l_z_);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Failed to capture initial FK");
    }
  }
  
  std::tuple<double, double, double, double, double, double>
  computeFootTargets(double cx, double cy, double alpha) {
    // Stance foot
    double sfx = steps_[step_].x, sfy = steps_[step_].y;
    
    // Swing foot
    int ni = std::min(step_ + 1, N_ - 1);
    int pi = std::max(0, step_ - 1);
    double sh = get_parameter("step_height").as_double();
    
    double swx, swy, swz;
    if (phase_ == DS) {
      swx = steps_[ni].x;
      swy = steps_[ni].y;
      swz = h_foot_;
    } else {
      double sm = alpha * alpha * (3 - 2 * alpha);
      swx = steps_[pi].x + sm * (steps_[ni].x - steps_[pi].x);
      swy = steps_[pi].y + sm * (steps_[ni].y - steps_[pi].y);
      swz = h_foot_ + sh * std::sin(M_PI * alpha);
    }
    
    // Transform walk coordinates to robot pelvis frame: robot_x = walk_y, robot_y = -walk_x
    double sf_robot_x = sfy;
    double sf_robot_y = -sfx;
    double sw_robot_x = swy;
    double sw_robot_y = -swx;
    
    // Walk frame to pelvis frame (add origin)
    double sf_px = sf_robot_x + walk_origin_x_;
    double sf_py = sf_robot_y + walk_origin_y_;
    double sw_px = sw_robot_x + walk_origin_x_;
    double sw_py = sw_robot_y + walk_origin_y_;
    
    double r_fx, r_fy, r_fz, l_fx, l_fy, l_fz;
    if (step_ % 2 == 0) {
      r_fx = sf_px;
      r_fy = sf_py;
      r_fz = h_foot_;
      l_fx = sw_px;
      l_fy = sw_py;
      l_fz = swz;
    } else {
      l_fx = sf_px;
      l_fy = sf_py;
      l_fz = h_foot_;
      r_fx = sw_px;
      r_fy = sw_py;
      r_fz = swz;
    }
    
    return {r_fx, r_fy, r_fz, l_fx, l_fy, l_fz};
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Joint Command Generation
  // ───────────────────────────────────────────────────────────────────────
  void legAnglesToJoints(const LegAngles &r, const LegAngles &l,
                         std::array<double, 12> &j) {
    auto cr = crouchAngles();
    // FK uses physical order: q(0)=hip_roll, q(1)=hip_pitch
    // Map to jointNames order: j[0]=hip_pitch, j[1]=hip_roll
    j[0] = -r.hip_pitch;   // hip_pitch_r (negated for URDF convention)
    j[1] = r.hip_roll;     // hip_roll_r
    j[2] = 0.0;            // thigh_yaw
    j[3] = -r.knee_pitch;  // knee_pitch_r (negated for URDF convention)
    j[4] = r.ankle_pitch;  // ankle_pitch_r
    j[5] = r.ankle_roll;   // ankle_roll_r
    
    j[6] = -l.hip_pitch;   // hip_pitch_l (negated for URDF convention)
    j[7] = l.hip_roll;     // hip_roll_l
    j[8] = 0.0;            // thigh_yaw
    j[9] = -l.knee_pitch;  // knee_pitch_l (negated for URDF convention)
    j[10] = l.ankle_pitch; // ankle_pitch_l
    j[11] = l.ankle_roll;  // ankle_roll_l
  }
  
  void applyFeedback(std::array<double, 12> &j, double total_force) {
    double kp = get_parameter("kp_pitch").as_double();
    if (kp > 0.0 && imu_ok_) {
      double kd = get_parameter("kd_pitch").as_double();
      double mc = get_parameter("max_corr").as_double();
      double pc = -(kp * imu_pitch_ + kd * imu_pitch_rate_);
      pc = std::max(-mc, std::min(mc, pc));
      j[4] += pc;
      j[10] += pc;
    }
    
    double kr = get_parameter("kp_roll").as_double();
    if (kr > 0.0 && imu_ok_) {
      double kdr = get_parameter("kd_roll").as_double();
      double mc = get_parameter("max_corr").as_double();
      double rc = -(kr * imu_roll_ + kdr * imu_roll_rate_);
      rc = std::max(-mc, std::min(mc, rc));
      j[5] += rc;
      j[11] -= rc;
    }
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Publishing & Logging
  // ───────────────────────────────────────────────────────────────────────
  void publishAngles(const std::array<double, 12> &a, double horizon) {
    trajectory_msgs::msg::JointTrajectory m;
    m.header.stamp = rclcpp::Time(0);
    auto n = jointNames();
    m.joint_names.assign(n.begin(), n.end());
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions.assign(a.begin(), a.end());
    p.time_from_start = rclcpp::Duration::from_seconds(horizon);
    m.points.push_back(p);
    walk_pub_->publish(m);
  }
  
  void publishMarkers(double cx, double cy, double zx, double zy) {
    visualization_msgs::msg::MarkerArray arr;
    auto now_t = now();
    
    auto mk = [&](int id, int type, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "pelvis";
      m.header.stamp = now_t;
      m.id = id;
      m.type = type;
      m.action = 0;
      m.pose.orientation.w = 1.0;
      m.color.a = 0.85f;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      return m;
    };
    
    // CoM commanded (blue) - already in robot frame
    {
      auto m = mk(0, 2, 0, 0.4f, 1.0f);
      m.pose.position.x = cx;
      m.pose.position.y = cy;
      m.pose.position.z = com_h_;
      m.scale.x = m.scale.y = m.scale.z = 0.06;
      arr.markers.push_back(m);
    }
    
    // ZMP reference (red) - already in robot frame from computeZMPReference
    {
      auto m = mk(2, 2, 1.0f, 0.1f, 0);
      m.pose.position.x = zx + walk_origin_x_;
      m.pose.position.y = zy + walk_origin_y_;
      m.pose.position.z = 0.02;
      m.scale.x = m.scale.y = m.scale.z = 0.05;
      arr.markers.push_back(m);
    }
    
    marker_pub_->publish(arr);
  }
  
  void logState(double alpha, double cx, double cy, double r_fx, double r_fy,
                double r_fz, double l_fx, double l_fy, double l_fz,
                const LegAngles &r, const LegAngles &l) {
    static rclcpp::Time last_log = now();
    if ((now() - last_log).seconds() < 0.5)
      return;
    last_log = now();
    
    const char *ps = phase_ == DS ? "DS" : phase_ == SL ? "SL" : "SR";
    RCLCPP_INFO(
        get_logger(),
        "[%d|%s|α=%.2f] CoM_cmd=(%.3f,%.3f)\n"
        "  R(%.3f,%.3f,%.3f) IK: hr=%.3f hp=%.3f kn=%.3f ap=%.3f ar=%.3f\n"
        "  L(%.3f,%.3f,%.3f) IK: hr=%.3f hp=%.3f kn=%.3f ap=%.3f ar=%.3f\n"
        "  DCM: ω=%.3f IMU: pitch=%.3f roll=%.3f",
        step_, ps, alpha, cx, cy, r_fx, r_fy, r_fz, r.hip_roll, r.hip_pitch,
        r.knee_pitch, r.ankle_pitch, r.ankle_roll, l_fx, l_fy, l_fz,
        l.hip_roll, l.hip_pitch, l.knee_pitch, l.ankle_pitch, l.ankle_roll,
        dcm_ctrl_->getOmega(), imu_pitch_, imu_roll_);
  }
  
  void resetWalk() {
    walk_active_ = false;
    walk_done_ = false;
    settle_ok_ = false;
    omega0_ok_ = false;
    step_ = 0;
    steps_.clear();
    traj_.clear();
    startup_ = now();
    if (dcm_ctrl_)
      dcm_ctrl_->reset();
    RCLCPP_INFO(get_logger(), "Walk reset");
  }
  
  // ───────────────────────────────────────────────────────────────────────
  // Member Variables
  // ───────────────────────────────────────────────────────────────────────
  enum Phase { DS, SL, SR };
  
  rclcpp::QoS qos_;
  rclcpp::Time startup_;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr fsub_[8];
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr walk_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
  rclcpp::TimerBase::SharedPtr com_timer_, ctrl_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  
  urdf::Model model_;
  std::map<std::string, double> link_mass_;
  std::map<std::string, std::vector<double>> link_com_;
  double total_mass_{0}, omega0_{0}, com_h_{0.7};
  double com_x_{0}, com_y_{0}, com_z_{0};
  double dcom_x_{0}, dcom_y_{0};
  double h_foot_{-0.659};
  bool urdf_ok_{false}, omega0_ok_{false};
  
  double imu_pitch_{0}, imu_roll_{0}, imu_pitch_rate_{0}, imu_roll_rate_{0};
  bool imu_ok_{false};
  ForceSensor fsens_[8];
  
  std::unique_ptr<DCMController> dcm_ctrl_;
  
  bool settle_ok_{false};
  bool walk_active_{false}, walk_done_{false};
  rclcpp::Time walk_start_, phase_start_;
  Phase phase_{DS};
  int step_{0}, N_{6};
  
  double walk_origin_x_{0}, walk_origin_y_{0};
  double foot_r_y_{0}, foot_l_y_{0};
  
  std::vector<Footstep> steps_;
  std::vector<TrajPoint> traj_;
  
  LegAngles r_ik_last_, l_ik_last_;
  double init_r_x_{0}, init_r_y_{0}, init_r_z_{0};
  double init_l_x_{0}, init_l_y_{0}, init_l_z_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DCMWalker>());
  rclcpp::shutdown();
  return 0;
}
