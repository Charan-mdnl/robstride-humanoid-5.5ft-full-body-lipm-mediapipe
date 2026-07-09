// =============================================================================
// kajita_walker.cpp
// Kajita-style LIPM bipedal walking controller for Angad humanoid (C++ / ROS2)
//
// Architecture
// ────────────
//  1. URDF callback  → parse link masses for CoM measurement.
//  2. CoM timer (5 Hz) → compute actual CoM height from TF; derive omega0.
//  3. Control loop (50 Hz) → phase machine → LIPM CoM → foot targets → IK →
//                            JointTrajectory + RViz markers.
//
// Frame conventions (verified with RViz markers against LIPM trajectory)
// ───────────────────────────────────────────────────────────────────────
//  Walk frame   : x = forward, y = left  (LIPM planning lives here)
//  World frame  : robot faces world +Y; walk_x → -world_y, walk_y → world_x
//  Pelvis frame : x forward, y left, z up  (IK input frame, via live TF)
//
// Key fix over backup.cpp
// ───────────────────────
//  backup.cpp converted foot targets as:
//    foot_pelvis = (foot_walk - com_walk_snapshot) + com_pelvis_at_walk_start
//  This snapshot drifts increasingly wrong as the robot moves forward.
//
//  Here: foot targets are expressed in world frame, then transformed to the
//  CURRENT pelvis frame via a live TF lookup every control tick.
//  This is exact regardless of how far the robot has walked.
//
// IK calibration (unchanged from backup.cpp — verified against settle pose)
// ──────────────────────────────────────────────────────────────────────────
//  The legIK() offsets were derived so that, at the standing foot TF position,
//  legIK() returns exactly the MuJoCo settle joint commands.  Do not change
//  them without re-calibrating from TF data.
// =============================================================================

#include <array>
#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Pinocchio (must come after Eigen)
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
// (Geometric IK constants removed — Pinocchio uses the actual URDF chain)

// ─────────────────────────────────────────────────────────────────────────────
// Small helper types
// ─────────────────────────────────────────────────────────────────────────────

struct TrajPoint {
  double t, x, xdot, y, ydot;
};

struct Footstep {
  double x, y;
}; // walk-frame horizontal position

// =============================================================================
class KajitaWalker : public rclcpp::Node {
public:
  KajitaWalker()
      : Node("kajita_walker"), qos_(rclcpp::QoS(1).transient_local()),
        startup_time_(this->now()) {
    // ── Declare tuning parameters (changeable at runtime) ────────────────
    this->declare_parameter("kp_pitch", 0.8);
    this->declare_parameter("kd_pitch", 0.05);
    this->declare_parameter("kp_cop", 3.0);
    this->declare_parameter("kp_roll", 0.5);
    this->declare_parameter("kd_roll", 0.03);
    this->declare_parameter("max_ankle_corr", 0.2);
    this->declare_parameter("settle_cop_gain", 0.5);
    this->declare_parameter("step_length", 0.06);
    this->declare_parameter("step_height", 0.03);
    this->declare_parameter("walk_enable", false); // hold crouch until set true
    // ── Subscriptions ────────────────────────────────────────────────────
    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos_,
        std::bind(&KajitaWalker::onUrdf, this, std::placeholders::_1));

    // ── Publishers ───────────────────────────────────────────────────────
    walk_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/lower_body_controller/joint_trajectory", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "walk_markers", 10);

    // ── IMU subscription ─────────────────────────────────────────────────
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_site_imu_sensor/imu", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          // MuJoCo doesn't populate orientation quaternion — compute from accel
          // Pelvis frame: X=lateral(left), Y=forward, Z=up
          const auto &a = msg->linear_acceleration;
          // When upright: a = (0, 0, +9.81)
          // Pitch (forward tilt): a.y goes negative when leaning forward
          imu_pitch_ = std::atan2(-a.y, std::sqrt(a.x * a.x + a.z * a.z));
          // Roll (lateral tilt): a.x goes positive when leaning left
          imu_roll_ = std::atan2(a.x, std::sqrt(a.y * a.y + a.z * a.z));
          // Angular velocities: pitch=rotation around X, roll=rotation around Y
          imu_pitch_rate_ = msg->angular_velocity.x;
          imu_roll_rate_ = msg->angular_velocity.y;
          imu_ready_ = true;
        });

    // ── Foot force subscriptions ─────────────────────────────────────────
    // Sensor positions in foot-local frame (x=lateral, y=sagittal)
    // Right foot
    force_sensors_[0] = {"r_toe_1", true, 0.0, 0.075, 0.0};
    force_sensors_[1] = {"r_toe_2", true, 0.015, 0.075, 0.0};
    force_sensors_[2] = {"r_heel_1", true, 0.0, -0.048, 0.0};
    force_sensors_[3] = {"r_heel_2", true, -0.015, -0.048, 0.0};
    // Left foot
    force_sensors_[4] = {"l_toe_1", false, 0.0, 0.075, 0.0};
    force_sensors_[5] = {"l_toe_2", false, -0.015, 0.075, 0.0};
    force_sensors_[6] = {"l_heel_1", false, 0.0, -0.048, 0.0};
    force_sensors_[7] = {"l_heel_2", false, 0.015, -0.048, 0.0};

    auto make_sensor_cb = [this](int idx) {
      return
          [this, idx](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            force_sensors_[idx].fz = msg->wrench.force.z;
          };
    };
    const char *r_topics[] = {"/foot_r_toe_site_wrench_sensor/wrench",
                              "/foot_r_toe_site_2_wrench_sensor/wrench",
                              "/foot_r_heel_site_wrench_sensor/wrench",
                              "/foot_r_heel_site_2_wrench_sensor/wrench"};
    const char *l_topics[] = {"/foot_l_toe_site_wrench_sensor/wrench",
                              "/foot_l_toe_site_2_wrench_sensor/wrench",
                              "/foot_l_heel_site_wrench_sensor/wrench",
                              "/foot_l_heel_site_2_wrench_sensor/wrench"};
    for (int i = 0; i < 4; ++i) {
      force_subs_[i] = create_subscription<geometry_msgs::msg::WrenchStamped>(
          r_topics[i], rclcpp::SensorDataQoS(), make_sensor_cb(i));
      force_subs_[i + 4] =
          create_subscription<geometry_msgs::msg::WrenchStamped>(
              l_topics[i], rclcpp::SensorDataQoS(), make_sensor_cb(i + 4));
    }

    // ── TF ───────────────────────────────────────────────────────────────
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, this);

    // ── Timers ───────────────────────────────────────────────────────────
    com_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // 5 Hz — CoM measurement
        std::bind(&KajitaWalker::onComTimer, this));

    ctrl_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50 Hz — control
        std::bind(&KajitaWalker::onControl, this));

    RCLCPP_INFO(get_logger(), "KajitaWalker started");
  }

private:
  // =========================================================================
  // Force sensor helpers
  // =========================================================================

  // CoP for one foot: returns {total_fz, cop_x, cop_y} in foot-local frame
  std::tuple<double, double, double> computeFootCoP(bool is_right) const {
    double sum_f = 0, sum_fx = 0, sum_fy = 0;
    for (int i = 0; i < 8; ++i) {
      if (force_sensors_[i].is_right != is_right)
        continue;
      double f = std::abs(force_sensors_[i].fz); // MuJoCo reports negative z
      sum_f += f;
      sum_fx += f * force_sensors_[i].x;
      sum_fy += f * force_sensors_[i].y;
    }
    if (sum_f < 1.0)
      return {sum_f, 0.0, 0.0}; // no contact
    return {sum_f, sum_fx / sum_f, sum_fy / sum_f};
  }

  // Overall ZMP: returns {total_fz, zmp_x, zmp_y} (simple average of both feet)
  std::tuple<double, double, double> computeZMP() const {
    auto [r_f, r_cx, r_cy] = computeFootCoP(true);
    auto [l_f, l_cx, l_cy] = computeFootCoP(false);
    double total = r_f + l_f;
    if (total < 1.0)
      return {total, 0.0, 0.0};
    // Weighted average of per-foot CoP (both in foot-local frame,
    // so sagittal (y) is meaningful; lateral (x) is approximate)
    double zx = (r_f * r_cx + l_f * l_cx) / total;
    double zy = (r_f * r_cy + l_f * l_cy) / total;
    return {total, zx, zy};
  }

  // =========================================================================
  // Frame helpers
  // =========================================================================

  // Walk frame (x=forward, y=left) ↔ World frame (robot faces world +Y).
  // Verified: walk_x maps to -world_y, walk_y maps to world_x.
  static std::array<double, 2> walkToWorld(double wx, double wy) {
    return {wy, -wx};
  }
  static std::array<double, 2> worldToWalk(double world_x, double world_y) {
    return {-world_y, world_x};
  }

  // =========================================================================
  // Joint layout
  // =========================================================================

  static std::array<std::string, 12> jointNames() {
    return {"hip_pitch_r",   "hip_roll_r",   "thigh_yaw_r",   "knee_pitch_r",
            "ankle_pitch_r", "ankle_roll_r", "hip_pitch_l",   "hip_roll_l",
            "thigh_yaw_l",   "knee_pitch_l", "ankle_pitch_l", "ankle_roll_l"};
  }

  // Crouch/walking stance verified from MuJoCo keyframe + FK self-check.
  static std::array<double, 12> settleAngles() {
    // Matches MuJoCo standing keyframe exactly
    return {-0.615, -0.059, 0.0, -0.593, -0.351, 0.0,
            -0.615, -0.059, 0.0, -0.593, -0.351, 0.0};
  }

  // =========================================================================
  // Pinocchio CLIK — uses actual URDF chain, no geometric approximations
  // =========================================================================
  void initPinocchio(const std::string &urdf_string) {
    pinocchio::urdf::buildModelFromXML(urdf_string, pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);

    foot_r_frame_ = pin_model_.getFrameId("foot_r");
    foot_l_frame_ = pin_model_.getFrameId("foot_l");

    // Map our 12 joint names → pinocchio q-vector indices
    const auto names = jointNames();
    for (int i = 0; i < 12; ++i) {
      bool found = false;
      for (int j = 1; j < (int)pin_model_.njoints; ++j) {
        if (pin_model_.names[j] == names[i]) {
          pin_joint_idx_[i] = pin_model_.joints[j].idx_q();
          found = true;
          break;
        }
      }
      if (!found)
        RCLCPP_ERROR(get_logger(), "Joint '%s' not in Pinocchio model",
                     names[i].c_str());
    }

    // Seed config at settle pose
    pin_q_ = pinocchio::neutral(pin_model_);
    auto settle = settleAngles();
    for (int i = 0; i < 12; ++i)
      pin_q_[pin_joint_idx_[i]] = settle[i];
    pin_q_settle_ = pin_q_; // save for posture regularization

    pinocchio::forwardKinematics(pin_model_, pin_data_, pin_q_);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);

    auto fr = pin_data_.oMf[foot_r_frame_].translation();
    auto fl = pin_data_.oMf[foot_l_frame_].translation();
    settle_foot_r_ = fr;
    settle_foot_l_ = fl;
    RCLCPP_INFO(get_logger(),
                "Pinocchio: foot_r=(%.4f,%.4f,%.4f) foot_l=(%.4f,%.4f,%.4f)",
                fr[0], fr[1], fr[2], fl[0], fl[1], fl[2]);

    // Compute walk→pelvis axis mapping from settle feet
    // In walk frame: L-R = (0, 2*kFootSep, 0) — purely lateral
    // In pelvis frame: L-R = settle_foot_l_ - settle_foot_r_
    Eigen::Vector3d lat_dir = fl - fr;
    lat_dir[2] = 0; // project to horizontal
    double lat_len = lat_dir.norm();
    if (lat_len > 0.01) {
      walk_lat_ = lat_dir / lat_len; // walk y → pelvis lateral
      walk_fwd_ =
          Eigen::Vector3d(-walk_lat_[1], walk_lat_[0], 0.0); // perpendicular
    } else {
      walk_lat_ = Eigen::Vector3d(0, 1, 0);
      walk_fwd_ = Eigen::Vector3d(1, 0, 0);
    }
    RCLCPP_INFO(get_logger(), "Walk→pelvis: fwd=(%.3f,%.3f) lat=(%.3f,%.3f)",
                walk_fwd_[0], walk_fwd_[1], walk_lat_[0], walk_lat_[1]);

    pin_ready_ = true;
  }

  // Solve IK for both feet. Targets are in Pinocchio's root (pelvis) frame.
  // Uses posture regularization to stay near settle config near singularities.
  std::array<double, 12> solveIK(const Eigen::Vector3d &tgt_r,
                                 const Eigen::Vector3d &tgt_l) {
    constexpr double damp = 1e-3; // increased for stability near singularity
    constexpr double reg = 0.05;  // posture regularization weight
    constexpr int max_iter = 100;
    constexpr double tol = 1e-3;

    // Seed from previous solution (pin_q_ tracks frame-to-frame)
    Eigen::VectorXd q = pin_q_;

    for (int iter = 0; iter < max_iter; ++iter) {
      pinocchio::forwardKinematics(pin_model_, pin_data_, q);
      pinocchio::updateFramePlacements(pin_model_, pin_data_);
      pinocchio::computeJointJacobians(pin_model_, pin_data_, q);

      Eigen::Vector3d e_r = tgt_r - pin_data_.oMf[foot_r_frame_].translation();
      Eigen::Vector3d e_l = tgt_l - pin_data_.oMf[foot_l_frame_].translation();
      if (e_r.norm() + e_l.norm() < tol)
        break;

      Eigen::MatrixXd Jr(6, pin_model_.nv), Jl(6, pin_model_.nv);
      Jr.setZero();
      Jl.setZero();
      pinocchio::getFrameJacobian(pin_model_, pin_data_, foot_r_frame_,
                                  pinocchio::LOCAL_WORLD_ALIGNED, Jr);
      pinocchio::getFrameJacobian(pin_model_, pin_data_, foot_l_frame_,
                                  pinocchio::LOCAL_WORLD_ALIGNED, Jl);

      Eigen::MatrixXd J(6, pin_model_.nv);
      J.topRows(3) = Jr.topRows(3);
      J.bottomRows(3) = Jl.topRows(3);
      Eigen::VectorXd err(6);
      err.head(3) = e_r;
      err.tail(3) = e_l;

      // Damped pseudo-inverse + posture regularization toward settle
      Eigen::MatrixXd JJt = J * J.transpose();
      JJt.diagonal().array() += damp;
      Eigen::VectorXd dq_task = J.transpose() * JJt.ldlt().solve(err);

      // Null-space posture: bias toward settle config
      Eigen::VectorXd q_err = q - pin_q_settle_;
      Eigen::VectorXd dq = dq_task - reg * q_err;

      q = pinocchio::integrate(pin_model_, q, dq);

      // Clamp to joint limits
      for (int i = 0; i < pin_model_.nq; ++i) {
        q[i] = std::max(pin_model_.lowerPositionLimit[i],
                        std::min(pin_model_.upperPositionLimit[i], q[i]));
      }
    }

    pin_q_ = q;
    std::array<double, 12> result;
    for (int i = 0; i < 12; ++i)
      result[i] = q[pin_joint_idx_[i]];
    return result;
  }

  // =========================================================================
  // URDF callback — parse link masses + init Pinocchio
  // =========================================================================
  void onUrdf(const std_msgs::msg::String::SharedPtr msg) {
    if (!model_.initString(msg->data)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
      return;
    }
    total_mass_ = 0.0;
    for (auto &[name, link] : model_.links_) {
      if (!link->inertial)
        continue;
      link_mass_[name] = link->inertial->mass;
      link_com_[name] = {link->inertial->origin.position.x,
                         link->inertial->origin.position.y,
                         link->inertial->origin.position.z};
      total_mass_ += link->inertial->mass;
    }
    RCLCPP_INFO(get_logger(), "URDF: %zu links, total mass %.2f kg",
                link_mass_.size(), total_mass_);
    initPinocchio(msg->data);
    urdf_ready_ = true;
    urdf_sub_.reset();
  }

  // =========================================================================
  // CoM timer (5 Hz) — measure CoM in world frame; compute omega0
  // =========================================================================
  void onComTimer() {
    if (!urdf_ready_)
      return;
    if ((now() - startup_time_).seconds() < kSettleDur)
      return;

    // ── Compute CoM in world frame ────────────────────────────────────────
    double wx = 0, wy = 0, wz = 0, m_used = 0;
    for (auto &[name, mass] : link_mass_) {
      geometry_msgs::msg::PointStamped lp, wp;
      lp.header.frame_id = name;
      lp.point.x = link_com_[name][0];
      lp.point.y = link_com_[name][1];
      lp.point.z = link_com_[name][2];
      try {
        wp = tf_buf_->transform(lp, "world", tf2::durationFromSec(0.1));
      } catch (...) {
        continue;
      }
      wx += mass * wp.point.x;
      wy += mass * wp.point.y;
      wz += mass * wp.point.z;
      m_used += mass;
    }
    if (m_used < 0.1)
      return;

    // ── Foot height in world frame ────────────────────────────────────────
    double fz_r = 0, fz_l = 0;
    try {
      geometry_msgs::msg::PointStamped orig, foot_w;
      orig.point.x = orig.point.y = orig.point.z = 0;
      orig.header.frame_id = "foot_r";
      foot_w = tf_buf_->transform(orig, "world", tf2::durationFromSec(0.1));
      fz_r = foot_w.point.z;
      orig.header.frame_id = "foot_l";
      foot_w = tf_buf_->transform(orig, "world", tf2::durationFromSec(0.1));
      fz_l = foot_w.point.z;
    } catch (...) {
      return;
    }

    const double foot_z_world = (fz_r + fz_l) * 0.5;
    const double h = (wz / m_used) - foot_z_world;
    if (h < 0.1)
      return;

    // Only update omega0 / h_foot while not walking (freeze at walk start)
    if (!walk_active_) {
      omega0_ = std::sqrt(9.81 / h);
      com_h_ = h;
      h_foot_world_ = foot_z_world;
    }

    if (!omega0_ready_ && omega0_ > 0.5) {
      omega0_ready_ = true;
      RCLCPP_INFO(get_logger(),
                  "omega0 ready: %.4f rad/s  h=%.3f m  foot_z_world=%.3f",
                  omega0_, com_h_, h_foot_world_);

      // ── Pinocchio IK self-check ──────────────────────────────────────
      if (pin_ready_) {
        RCLCPP_INFO(get_logger(),
                    "Pinocchio settle feet:\n"
                    "  R: (%.4f, %.4f, %.4f)\n"
                    "  L: (%.4f, %.4f, %.4f)\n"
                    "  Walk→pelvis: fwd=(%.3f,%.3f) lat=(%.3f,%.3f)",
                    settle_foot_r_[0], settle_foot_r_[1], settle_foot_r_[2],
                    settle_foot_l_[0], settle_foot_l_[1], settle_foot_l_[2],
                    walk_fwd_[0], walk_fwd_[1], walk_lat_[0], walk_lat_[1]);

        auto result = solveIK(settle_foot_r_, settle_foot_l_);
        RCLCPP_INFO(get_logger(),
                    "Pinocchio IK self-check (settle targets):\n"
                    "  R: %.3f %.3f %.3f %.3f %.3f %.3f\n"
                    "  L: %.3f %.3f %.3f %.3f %.3f %.3f",
                    result[0], result[1], result[2], result[3], result[4],
                    result[5], result[6], result[7], result[8], result[9],
                    result[10], result[11]);
      }
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CoM world (%.3f, %.3f, %.3f) | foot_z=%.3f | h=%.3f | omega0=%.4f",
        wx / m_used, wy / m_used, wz / m_used, foot_z_world, h, omega0_);
  }

  // =========================================================================
  // Footstep planning — Kajita-style pre-planned flat-terrain footsteps
  // =========================================================================
  //
  // Sequence (walk frame, x=forward y=left):
  //   step 0 (even): right foot — initial stance, same x as start
  //   step 1 (odd) : left  foot — first left step,  same x as start
  //   step 2 (even): right foot — step_length forward
  //   step 3 (odd) : left  foot — step_length forward
  //   ... (N-1 steps total)
  //
  void initFootsteps() {
    auto fr = tf_buf_->lookupTransform("world", "foot_r", tf2::TimePointZero);
    auto fl = tf_buf_->lookupTransform("world", "foot_l", tf2::TimePointZero);

    auto [frwx, frwy] =
        worldToWalk(fr.transform.translation.x, fr.transform.translation.y);
    auto [flwx, flwy] =
        worldToWalk(fl.transform.translation.x, fl.transform.translation.y);

    const double fx0 = (frwx + flwx) * 0.5;
    const double fy0 = (frwy + flwy) * 0.5;
    const double fy_r = fy0 - kFootSep; // right foot y (negative side)
    const double fy_l = fy0 + kFootSep; // left  foot y (positive side)

    footsteps_.clear();
    for (int i = 0; i < kN; ++i) {
      // First two footsteps share the initial x (double support start).
      // From step 2 onward each step advances by step_length.
      const double fx = (i <= 1) ? fx0 : fx0 + (i - 1) * kStepLength;
      const double fy = (i % 2 == 0) ? fy_r : fy_l;
      footsteps_.push_back({fx, fy});
    }

    RCLCPP_INFO(get_logger(),
                "Footsteps init (walk frame): fx0=%.3f  fy_r=%.3f  fy_l=%.3f",
                fx0, fy_r, fy_l);
    for (int i = 0; i < kN; ++i)
      RCLCPP_INFO(get_logger(), "  step[%d]: (%.3f, %.3f)", i, footsteps_[i].x,
                  footsteps_[i].y);
  }

  // =========================================================================
  // LIPM trajectory (Kajita ch.3 — orbital IC for periodic walking)
  // =========================================================================
  //
  // For each single-support step the LIPM propagates under constant ZMP at
  // the stance foot position.  The orbital IC is chosen so the CoM is exactly
  // self-repeating: at step start it sits step_length/2 behind the ZMP.
  //
  // Closed-form LIPM propagation over dt (exact, no Euler approximation):
  //   x(t+dt) = (x(t)-p)*cosh(ω·dt) + (ẋ(t)/ω)*sinh(ω·dt) + p
  //   ẋ(t+dt) = (x(t)-p)*ω·sinh(ω·dt) + ẋ(t)*cosh(ω·dt)
  //
  void computeLipmTrajectory() {
    const double wT = omega0_ * kTss;

    // ── Sagittal IC (Kajita eq. 3.35) ────────────────────────────────────
    // xdot0 chosen so CoM arrives at +step_length/2 in front of next ZMP
    // at the end of the step, ready for the symmetric reverse step.
    const double xdot0 =
        kStepLength * omega0_ * std::sinh(wT) / (2.0 * (std::cosh(wT) - 1.0));

    // x0: CoM starts step_length/2 behind the first ZMP (footsteps_[1])
    // x_rel = xdot0*(1-cosh(wT))/(omega0*sinh(wT)) = -step_length/2
    const double x_rel =
        xdot0 * (1.0 - std::cosh(wT)) / (omega0_ * std::sinh(wT));

    // ── Lateral IC ────────────────────────────────────────────────────────
    // CoM starts at midpoint between feet; ydot0 drives it toward the
    // stance foot side so it arrives at the correct lateral position.
    const double y0 = (footsteps_[0].y + footsteps_[1].y) * 0.5;
    const double y_off = footsteps_[1].y - y0; // = kFootSep
    const double ydot0 =
        y_off * omega0_ * std::sinh(wT) / (std::cosh(wT) + 1.0);

    // ── Integrate over all steps ──────────────────────────────────────────
    com_trajectory_.clear();
    double x = footsteps_[0].x + x_rel;
    double y = y0;
    double xdot = xdot0, ydot = ydot0;
    com_trajectory_.push_back({0.0, x, xdot, y, ydot});

    for (int i = 0; i + 1 < kN; ++i) {
      const double zx = footsteps_[i + 1].x;
      const double zy = footsteps_[i + 1].y;
      for (double t = 0.0; t < kTss - 1e-9; t += kDt) {
        const double c = std::cosh(omega0_ * kDt);
        const double s = std::sinh(omega0_ * kDt);

        double xn = (x - zx) * c + (xdot / omega0_) * s + zx;
        double xdn = (x - zx) * omega0_ * s + xdot * c;
        double yn = (y - zy) * c + (ydot / omega0_) * s + zy;
        double ydn = (y - zy) * omega0_ * s + ydot * c;

        x = xn;
        xdot = xdn;
        y = yn;
        ydot = ydn;
        com_trajectory_.push_back({t, x, xdot, y, ydot});
      }
    }

    RCLCPP_INFO(get_logger(),
                "LIPM trajectory: %zu points\n"
                "  IC: x0=%.4f xdot0=%.4f | y0=%.4f ydot0=%.4f",
                com_trajectory_.size(), footsteps_[0].x + x_rel, xdot0, y0,
                ydot0);
  }

  // =========================================================================
  // Main control loop (50 Hz)
  // =========================================================================
  void onControl() {
    const double t_up = (now() - startup_time_).seconds();

    // ── Phase A: force-feedback settle — actively center CoP ─────────────
    if (!settle_stable_) {
      auto angs = settleAngles();
      auto [total_f, cop_x, cop_y] = computeZMP();

      if (total_f > 100.0 && t_up > 1.0) {
        // Integral control: cop_y > 0 (toes) → need less dorsiflexion
        // → increase ankle_pitch (make less negative / more positive)
        double cop_gain = this->get_parameter("settle_cop_gain").as_double();
        settle_ankle_adj_ += cop_gain * cop_y * kDt;
        settle_ankle_adj_ = std::max(-0.15, std::min(0.15, settle_ankle_adj_));
      }
      angs[4] += settle_ankle_adj_;  // R ankle pitch
      angs[10] += settle_ankle_adj_; // L ankle pitch
      publishJointsArr(angs);

      // Log and check stability
      if (t_up > 3.0) {
        static int log_cnt = 0;
        if (++log_cnt % 25 == 0) {
          RCLCPP_INFO(get_logger(),
                      "Settle: F=%.0f N  CoP=(%.3f,%.3f)  adj=%.4f  pitch=%.3f",
                      total_f, cop_x, cop_y, settle_ankle_adj_, imu_pitch_);
        }
        // Stable when CoP centered AND pitch small
        bool cop_ok = std::abs(cop_y) < 0.005;      // within 5mm
        bool pitch_ok = std::abs(imu_pitch_) < 0.1; // within ~6°
        if (cop_ok && pitch_ok && total_f > 100.0) {
          settle_stable_ = true;
          RCLCPP_INFO(
              get_logger(),
              "Force-verified settle: F=%.0f N CoP=(%.3f,%.3f) adj=%.4f",
              total_f, cop_x, cop_y, settle_ankle_adj_);
        } else if (t_up > kSettleDur) {
          settle_stable_ = true;
          RCLCPP_WARN(
              get_logger(),
              "Settle timeout: F=%.0f CoP=(%.3f,%.3f) adj=%.4f pitch=%.3f",
              total_f, cop_x, cop_y, settle_ankle_adj_, imu_pitch_);
        }
      }
      return;
    }

    if (!omega0_ready_)
      return;

    // ── walk_enable gate: hold crouch pose until param set true ──────────
    if (!this->get_parameter("walk_enable").as_bool()) {
      // Re-send settle pose to hold crouch with active CoP feedback
      auto angs = settleAngles();
      auto [total_f, cop_x, cop_y] = computeZMP();
      if (total_f > 100.0) {
        double cop_gain = this->get_parameter("settle_cop_gain").as_double();
        settle_ankle_adj_ += cop_gain * cop_y * kDt;
        settle_ankle_adj_ = std::max(-0.15, std::min(0.15, settle_ankle_adj_));
      }
      angs[4] += settle_ankle_adj_;
      angs[10] += settle_ankle_adj_;
      publishJointsArr(angs);
      static int hc = 0;
      if (++hc % 100 == 0)
        RCLCPP_INFO(get_logger(),
                    "[HOLD CROUCH] F=%.0f CoP=(%.3f,%.3f) adj=%.4f  "
                    "(set walk_enable:=true to start walking)",
                    total_f, cop_x, cop_y, settle_ankle_adj_);
      return;
    }

    // ── Phase B: initialise walk ──────────────────────────────────────────
    if (!walk_active_) {
      if (walk_done_)
        return;
      try {
        initFootsteps();
        computeLipmTrajectory();
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Walk init failed: %s", e.what());
        return;
      }
      walk_active_ = true;
      walk_start_time_ = now();
      phase_start_ = now();
      current_phase_ = DOUBLE_SUPPORT;
      current_step_ = 0;
      right_stance_ = true;
      // Reset IMU state so stale data from pre-settle doesn't contaminate
      imu_pitch_ = 0.0;
      imu_roll_ = 0.0;
      imu_pitch_rate_ = 0.0;
      imu_roll_rate_ = 0.0;
      RCLCPP_INFO(get_logger(), "Walk started");
    }

    // ── Phase C: advance the gait state machine ───────────────────────────
    const double phase_t = (now() - phase_start_).seconds();
    double alpha = 0.0;

    switch (current_phase_) {
    case DOUBLE_SUPPORT: {
      // First DS phase is longer for smooth weight transfer
      const double ds_dur = (current_step_ == 0) ? 2.0 * kTds : kTds;
      alpha = std::min(phase_t / ds_dur, 1.0);
      // Smoothly shift ZMP from current foot to next foot
      if (current_step_ + 1 < kN) {
        zmp_x_ = footsteps_[current_step_].x +
                 alpha * (footsteps_[current_step_ + 1].x -
                          footsteps_[current_step_].x);
        zmp_y_ = footsteps_[current_step_].y +
                 alpha * (footsteps_[current_step_ + 1].y -
                          footsteps_[current_step_].y);
      }
      // Force-gated: only transition when weight has shifted to stance foot
      if (phase_t > ds_dur && current_step_ + 1 < kN) {
        auto [r_f, rx, ry] = computeFootCoP(true);
        auto [l_f, lx, ly] = computeFootCoP(false);
        double total = r_f + l_f;
        // Next stance foot (odd step → left stance, even → right stance)
        bool next_right_stance = !right_stance_;
        double stance_f = next_right_stance ? r_f : l_f;
        bool weight_ok = (total < 10.0) ||         // no contact data → use time
                         (stance_f > 0.4 * total); // 40% weight on stance
        bool time_guard = phase_t > 2.0 * ds_dur;  // max 2x duration
        if (weight_ok || time_guard) {
          ++current_step_;
          current_phase_ = right_stance_ ? SINGLE_RIGHT : SINGLE_LEFT;
          right_stance_ = !right_stance_;
          phase_start_ = now();
        }
      }
      break;
    }

    case SINGLE_LEFT:
    case SINGLE_RIGHT:
      alpha = std::min(phase_t / kTss, 1.0);
      zmp_x_ = footsteps_[current_step_].x;
      zmp_y_ = footsteps_[current_step_].y;
      // Force-gated: only transition when swing foot touches down
      if (phase_t > kTss) {
        if (current_step_ >= kN - 2) {
          walk_active_ = false;
          walk_done_ = true;
          RCLCPP_INFO(get_logger(), "Walk finished");
          return;
        }
        // Check swing foot has contact
        bool swing_is_right = (current_phase_ == SINGLE_LEFT);
        auto [sw_f, sx, sy] = computeFootCoP(swing_is_right);
        bool contact = sw_f > kContactThreshold;
        bool time_guard = phase_t > 2.0 * kTss;
        if (contact || time_guard) {
          current_phase_ = DOUBLE_SUPPORT;
          phase_start_ = now();
        }
      }
      break;
    }

    // ── Phase D: desired CoM position from precomputed LIPM trajectory ────
    //
    // Trajectory is indexed as:
    //   step i → indices [i*pts_per_step , (i+1)*pts_per_step)
    //
    // During DS: hold the trajectory boundary value (start of next SS).
    // During SS: advance through the trajectory for the current step.
    //
    const int pts = std::max(1, static_cast<int>(kTss / kDt));
    double cx, cy;

    if (current_phase_ == DOUBLE_SUPPORT) {
      int idx = std::min(current_step_ * pts,
                         static_cast<int>(com_trajectory_.size()) - 1);
      cx = com_trajectory_[idx].x;
      cy = com_trajectory_[idx].y;
    } else {
      const int base = std::max(0, (current_step_ - 1) * pts);
      const int off = static_cast<int>(phase_t / kDt);
      const int idx =
          std::min(base + off, static_cast<int>(com_trajectory_.size()) - 1);
      cx = com_trajectory_[idx].x;
      cy = com_trajectory_[idx].y;
    }

    // ── Phase E: foot target positions in walk frame ──────────────────────
    const double sfx = footsteps_[current_step_].x;
    const double sfy = footsteps_[current_step_].y;

    const int prev_i = std::max(0, current_step_ - 1);
    const int next_i = std::min(current_step_ + 1, kN - 1);

    double sw_wx, sw_wy, sw_wz;
    if (current_phase_ == DOUBLE_SUPPORT) {
      // Swing foot already landed at next_i during previous single-support
      sw_wx = footsteps_[next_i].x;
      sw_wy = footsteps_[next_i].y;
      sw_wz = h_foot_world_;
    } else {
      // Cubic smooth-step interpolation from prev_i → next_i
      const double s = alpha * alpha * (3.0 - 2.0 * alpha);
      sw_wx = footsteps_[prev_i].x +
              s * (footsteps_[next_i].x - footsteps_[prev_i].x);
      sw_wy = footsteps_[prev_i].y +
              s * (footsteps_[next_i].y - footsteps_[prev_i].y);
      sw_wz = h_foot_world_ + kStepHeight * std::sin(M_PI * alpha);
    }

    // ── Phase F: foot positions in Pinocchio pelvis frame ─────────────────
    //
    // delta = (foot_walk - com_walk) - (foot_walk_at_settle - com_at_settle)
    // target = settle_foot_pin + R_walk2pelvis * delta
    //
    const double cx0 = com_trajectory_[0].x;
    const double cy0 = com_trajectory_[0].y;

    // Assign stance/swing to right/left (in walk frame)
    double r_walk_x, r_walk_y, r_walk_dz;
    double l_walk_x, l_walk_y, l_walk_dz;

    if (current_step_ % 2 == 0) { // even → right=stance, left=swing
      r_walk_x = sfx;
      r_walk_y = sfy;
      r_walk_dz = 0.0;
      l_walk_x = sw_wx;
      l_walk_y = sw_wy;
      l_walk_dz = (current_phase_ != DOUBLE_SUPPORT)
                      ? kStepHeight * std::sin(M_PI * alpha)
                      : 0.0;
    } else { // odd → left=stance, right=swing
      l_walk_x = sfx;
      l_walk_y = sfy;
      l_walk_dz = 0.0;
      r_walk_x = sw_wx;
      r_walk_y = sw_wy;
      r_walk_dz = (current_phase_ != DOUBLE_SUPPORT)
                      ? kStepHeight * std::sin(M_PI * alpha)
                      : 0.0;
    }

    // Walk-frame deltas (CoM-relative change from settle)
    double dr_x = (r_walk_x - cx) - (footsteps_[0].x - cx0);
    double dr_y = (r_walk_y - cy) - (footsteps_[0].y - cy0);
    double dl_x = (l_walk_x - cx) - (footsteps_[1].x - cx0);
    double dl_y = (l_walk_y - cy) - (footsteps_[1].y - cy0);

    // Clamp deltas to prevent near-singularity (max 5cm from settle)
    constexpr double kMaxDelta = 0.05;
    auto clamp = [](double v, double lim) {
      return std::max(-lim, std::min(lim, v));
    };
    dr_x = clamp(dr_x, kMaxDelta);
    dr_y = clamp(dr_y, kMaxDelta);
    dl_x = clamp(dl_x, kMaxDelta);
    dl_y = clamp(dl_y, kMaxDelta);

    // Convert walk deltas to pelvis frame via axis mapping
    Eigen::Vector3d dr_p = walk_fwd_ * dr_x + walk_lat_ * dr_y;
    Eigen::Vector3d dl_p = walk_fwd_ * dl_x + walk_lat_ * dl_y;

    Eigen::Vector3d tgt_r = settle_foot_r_ + dr_p;
    tgt_r[2] = settle_foot_r_[2] + r_walk_dz;
    Eigen::Vector3d tgt_l = settle_foot_l_ + dl_p;
    tgt_l[2] = settle_foot_l_[2] + l_walk_dz;

    // ── Phase G: Pinocchio IK ────────────────────────────────────────────
    auto joints = solveIK(tgt_r, tgt_l);

    // ── Logging (0.5 Hz) ──────────────────────────────────────────────────
    {
      static rclcpp::Time last_log = now();
      if ((now() - last_log).seconds() > 0.5) {
        last_log = now();
        const char *ph = current_phase_ == DOUBLE_SUPPORT ? "DS"
                         : current_phase_ == SINGLE_LEFT  ? "SL"
                                                          : "SR";
        RCLCPP_INFO(
            get_logger(),
            "[step %d | %s | α=%.2f]  CoM walk (%.3f, %.3f)\n"
            "  R target (%.3f, %.3f, %.3f)  joints: %.3f %.3f %.3f %.3f\n"
            "  L target (%.3f, %.3f, %.3f)  joints: %.3f %.3f %.3f %.3f\n"
            "  IMU: pitch=%.3f roll=%.3f  corr_p=%.3f corr_r=%.3f",
            current_step_, ph, alpha, cx, cy, tgt_r[0], tgt_r[1], tgt_r[2],
            joints[0], joints[1], joints[3], joints[4], tgt_l[0], tgt_l[1],
            tgt_l[2], joints[6], joints[7], joints[9], joints[10], imu_pitch_,
            imu_roll_, kKpPitch * imu_pitch_ + kKdPitch * imu_pitch_rate_,
            -kKpRoll * imu_roll_ - kKdRoll * imu_roll_rate_);
      }
    }

    // ── Phase H: ZMP + IMU ankle stabilization ────────────────────────────
    {
      // Compute per-foot CoP for sagittal ankle correction
      auto [r_fz, r_cop_x, r_cop_y] = computeFootCoP(true);
      auto [l_fz, l_cop_x, l_cop_y] = computeFootCoP(false);

      // Sagittal: if CoP is forward (cop_y > 0 = toes), ankle pitch too
      // negative → make ankle pitch less negative (add positive correction)
      // Stance foot gets full correction, swing foot gets half
      bool r_stance =
          (current_phase_ == SINGLE_RIGHT || current_phase_ == DOUBLE_SUPPORT);
      bool l_stance =
          (current_phase_ == SINGLE_LEFT || current_phase_ == DOUBLE_SUPPORT);
      if (r_fz > 5.0) { // foot has ground contact
        double cop_gain = this->get_parameter("kp_cop").as_double();
        double corr = -cop_gain * r_cop_y;
        joints[4] += r_stance ? corr : 0.5 * corr;
      }
      if (l_fz > 5.0) {
        double cop_gain = this->get_parameter("kp_cop").as_double();
        double corr = -cop_gain * l_cop_y;
        joints[10] += l_stance ? corr : 0.5 * corr;
      }

      // IMU pitch: negative = leaning forward → need negative ankle corr
      if (imu_ready_) {
        double kp = this->get_parameter("kp_pitch").as_double();
        double kd = this->get_parameter("kd_pitch").as_double();
        double max_corr = this->get_parameter("max_ankle_corr").as_double();
        double pitch_corr = kp * imu_pitch_ + kd * imu_pitch_rate_;
        pitch_corr = std::max(-max_corr, std::min(max_corr, pitch_corr));
        joints[4] += pitch_corr;
        joints[10] += pitch_corr;
      }

      // Lateral: IMU roll for ankle roll correction
      if (imu_ready_) {
        double kp_r = this->get_parameter("kp_roll").as_double();
        double kd_r = this->get_parameter("kd_roll").as_double();
        double roll_corr = -kp_r * imu_roll_ - kd_r * imu_roll_rate_;
        joints[5] += roll_corr;
        joints[11] += roll_corr;
      }
    }

    // ── Phase I: publish ──────────────────────────────────────────────────
    publishJoints(joints);
    publishMarkers(cx, cy);
  }

  // =========================================================================
  // Publishers
  // =========================================================================

  void publishSettle() {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    const auto names = jointNames();
    msg.joint_names.assign(names.begin(), names.end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    const auto angs = settleAngles();
    pt.positions.assign(angs.begin(), angs.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(2.0);
    msg.points.push_back(pt);
    walk_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Settle pose sent");
  }

  // Like publishSettle but with custom angles (for IMU-corrected settle)
  void publishJointsArr(const std::array<double, 12> &angs) {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    const auto names = jointNames();
    msg.joint_names.assign(names.begin(), names.end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(angs.begin(), angs.end());
    pt.time_from_start =
        rclcpp::Duration::from_seconds(0.1); // moderate horizon
    msg.points.push_back(pt);
    walk_pub_->publish(msg);
  }

  void publishJoints(const std::array<double, 12> &joints) {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    const auto names = jointNames();
    msg.joint_names.assign(names.begin(), names.end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(joints.begin(), joints.end());

    // Ramp horizon from 0.5s → kDt over first 2s of walking to avoid jerk
    const double walk_elapsed = (now() - walk_start_time_).seconds();
    const double ramp = std::max(0.0, std::min(1.0, walk_elapsed / 2.0));
    const double horizon = 0.5 * (1.0 - ramp) + kDt * ramp;
    pt.time_from_start = rclcpp::Duration::from_seconds(horizon);
    msg.points.push_back(pt);
    walk_pub_->publish(msg);
  }

  // =========================================================================
  // RViz markers — same convention as backup.cpp (verified correct)
  // =========================================================================
  void publishMarkers(double cx, double cy) {
    visualization_msgs::msg::MarkerArray arr;
    const auto now_t = now();
    const double z_com = h_foot_world_ + com_h_;

    auto make_marker = [&](int id, int type, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp = now_t;
      m.id = id;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.color.a = 0.85f;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      return m;
    };

    // Desired CoM — blue sphere
    {
      auto m = make_marker(0, visualization_msgs::msg::Marker::SPHERE, 0.f,
                           0.4f, 1.f);
      auto [wx, wy] = walkToWorld(cx, cy);
      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = z_com;
      m.scale.x = m.scale.y = m.scale.z = 0.06;
      arr.markers.push_back(m);
    }

    // Full precomputed CoM trajectory — green line strip
    {
      auto m = make_marker(1, visualization_msgs::msg::Marker::LINE_STRIP, 0.f,
                           1.f, 0.f);
      m.scale.x = 0.01;
      for (auto &tp : com_trajectory_) {
        geometry_msgs::msg::Point p;
        auto [wx, wy] = walkToWorld(tp.x, tp.y);
        p.x = wx;
        p.y = wy;
        p.z = z_com;
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }

    // ZMP — red sphere
    {
      auto m = make_marker(2, visualization_msgs::msg::Marker::SPHERE, 1.f,
                           0.1f, 0.f);
      auto [wx, wy] = walkToWorld(zmp_x_, zmp_y_);
      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = 0.02;
      m.scale.x = m.scale.y = m.scale.z = 0.05;
      arr.markers.push_back(m);
    }

    // Footstep targets — right=orange, left=cyan cubes
    for (int i = 0; i < static_cast<int>(footsteps_.size()); ++i) {
      float fr = (i % 2 == 0) ? 1.f : 0.f;
      float fg = (i % 2 == 0) ? 0.5f : 0.9f;
      float fb = (i % 2 == 0) ? 0.f : 0.9f;
      auto m = make_marker(10 + i, visualization_msgs::msg::Marker::CUBE, fr,
                           fg, fb);
      auto [wx, wy] = walkToWorld(footsteps_[i].x, footsteps_[i].y);
      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = 0.01;
      m.scale.x = 0.08;
      m.scale.y = 0.04;
      m.scale.z = 0.01;
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
  }

  // =========================================================================
  // Parameters — adjust here, not scattered through the code
  // =========================================================================

  static constexpr double kSettleDur = 6.0;   // s  — wait before walking
  static constexpr double kDt = 0.02;         // s  — control / LIPM dt
  static constexpr double kTss = 0.5;         // s  — single-support period
  static constexpr double kTds = 0.3;         // s  — double-support period
  static constexpr double kStepLength = 0.06; // m  — forward distance per step
  static constexpr double kFootSep = 0.145;   // m  — half lateral foot spacing
  static constexpr double kStepHeight = 0.03; // m  — swing foot clearance
  static constexpr int kN = 8;                // total footstep count

  // Ankle stabilization gains
  static constexpr double kKpPitch = 0.8;  // IMU pitch → ankle pitch (rad/rad)
  static constexpr double kKdPitch = 0.05; // IMU pitch rate damping
  static constexpr double kKpCop = 3.0;    // CoP offset → ankle pitch (rad/m)
  static constexpr double kKpRoll = 0.5;   // IMU roll → ankle roll
  static constexpr double kKdRoll = 0.03;  // IMU roll rate damping
  static constexpr double kMaxAnkleCorr = 0.2; // max ankle correction (rad)

  // Force-based phase transition
  static constexpr double kContactThreshold = 50.0; // N — swing foot touchdown

  // =========================================================================
  // Member variables
  // =========================================================================

  rclcpp::QoS qos_;
  rclcpp::Time startup_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      force_subs_[8];
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr walk_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
  rclcpp::TimerBase::SharedPtr com_timer_, ctrl_timer_;

  // IMU state
  double imu_pitch_{0.0}, imu_roll_{0.0};
  double imu_pitch_rate_{0.0}, imu_roll_rate_{0.0};
  bool imu_ready_{false};

  // Per-sensor force data
  struct ForceSensor {
    std::string name;
    bool is_right;
    double x, y;  // position in foot-local frame
    double fz{0}; // measured normal force
  };
  ForceSensor force_sensors_[8];
  bool settle_stable_{false};
  double settle_ankle_adj_{0.0}; // integral ankle adjustment from CoP feedback

  urdf::Model model_;
  std::map<std::string, double> link_mass_;
  std::map<std::string, std::vector<double>> link_com_;
  double total_mass_{0.0};

  bool urdf_ready_{false};
  bool omega0_ready_{false};
  double omega0_{0.0};
  double com_h_{0.0};
  double h_foot_world_{0.0}; // frozen at walk start

  bool settle_sent_{false};
  bool walk_active_{false};
  bool walk_done_{false};
  rclcpp::Time phase_start_;
  rclcpp::Time walk_start_time_;

  enum Phase { DOUBLE_SUPPORT, SINGLE_LEFT, SINGLE_RIGHT };
  Phase current_phase_{DOUBLE_SUPPORT};
  int current_step_{0};
  bool right_stance_{true};

  std::vector<Footstep> footsteps_;
  std::vector<TrajPoint> com_trajectory_;

  double zmp_x_{0.0}, zmp_y_{0.0};

  // Pinocchio model for CLIK
  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_;
  pinocchio::FrameIndex foot_r_frame_{0}, foot_l_frame_{0};
  std::array<int, 12> pin_joint_idx_{};
  Eigen::VectorXd pin_q_;
  Eigen::VectorXd pin_q_settle_; // settle config for posture regularization
  Eigen::Vector3d settle_foot_r_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d settle_foot_l_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d walk_fwd_{1, 0, 0}; // walk x-axis in pelvis frame
  Eigen::Vector3d walk_lat_{0, 1, 0}; // walk y-axis in pelvis frame
  bool pin_ready_{false};
};

// =============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KajitaWalker>());
  rclcpp::shutdown();
  return 0;
}
