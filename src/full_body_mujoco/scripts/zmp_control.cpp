// ZMP Walking Controller with Force/IMU Feedback
// Based on proven geometric IK from backup.cpp
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

class ZmpWalker : public rclcpp::Node {
public:
  ZmpWalker()
      : Node("zmp_walker"), qos_(rclcpp::QoS(1).transient_local()),
        startup_(now()) {
    // Runtime-tunable parameters
    declare_parameter("kp_pitch", 0.0); // disabled by default
    declare_parameter("kd_pitch", 0.0);
    declare_parameter("kp_cop", 0.0);  // disabled by default
    declare_parameter("kp_roll", 0.0); // disabled by default
    declare_parameter("kd_roll", 0.0);
    declare_parameter("com_sway", 0.055); // 5.5cm over-shift to counter heavy swing leg
    declare_parameter("max_corr", 0.15);
    declare_parameter("settle_gain", 0.05); // gentle CoP correction
    declare_parameter("step_length", 0.05); // 5cm forward steps
    declare_parameter("step_height", 0.03); // 3cm lift
    declare_parameter("t_ss", 2.0); // Ultra-conservative single support
    declare_parameter("t_ds", 1.5); // Slow double support weight transfer
    declare_parameter("reset",
                      false); // set true to restart walk with new params

    // Watch for param changes — reset triggers new walk
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
          for (auto &p : params) {
            if (p.get_name() == "reset" && p.as_bool()) {
              walk_active_ = false;
              walk_done_ = false;
              settle_ok_ = false;
              omega0_ok_ = false;
              settle_adj_ = 0;
              step_ = 0;
              steps_.clear();
              traj_.clear();
              startup_ = now();
              RCLCPP_INFO(get_logger(), "Reset! Will re-settle and re-walk.");
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
        [this](sensor_msgs::msg::Imu::SharedPtr m) {
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
        });

    // Force sensors (8 total, 4 per foot)
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
      auto cb = [this, i](geometry_msgs::msg::WrenchStamped::SharedPtr m) {
        fsens_[i].fz = m->wrench.force.z;
      };
      auto cb2 = [this, i](geometry_msgs::msg::WrenchStamped::SharedPtr m) {
        fsens_[i + 4].fz = m->wrench.force.z;
      };
      fsub_[i] = create_subscription<geometry_msgs::msg::WrenchStamped>(
          rt[i], rclcpp::SensorDataQoS(), cb);
      fsub_[i + 4] = create_subscription<geometry_msgs::msg::WrenchStamped>(
          lt[i], rclcpp::SensorDataQoS(), cb2);
    }

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

    RCLCPP_INFO(get_logger(), "ZmpWalker started");
  }

private:
  // ── Force helpers ─────────────────────────────────────────────────────
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
    return {t, (rf * rx + lf * lx) / t, (rf * ry + lf * ly) / t};
  }

  // ── Numerical IK: Kajita et al. "Introduction to Humanoid Robotics" ────────
  // rot2omega: rotation matrix → angular velocity vector (§2.5.9 Fig. 2.40)
  static Eigen::Vector3d rot2omega(const Eigen::Matrix3d &R) {
    Eigen::Vector3d el(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
    double n = el.norm();
    if (n > 1e-10)
      return std::atan2(n, R.trace() - 1.0) / n * el;
    if (R(0,0) > 0 && R(1,1) > 0 && R(2,2) > 0)
      return Eigen::Vector3d::Zero();
    return (M_PI / 2.0) * Eigen::Vector3d(R(0,0)+1, R(1,1)+1, R(2,2)+1);
  }

  // calcVWerr: 6-D position + orientation error vector (§2.5.9 Fig. 2.39)
  static Eigen::Matrix<double,6,1> calcVWerr(
      const Eigen::Vector3d &p_ref, const Eigen::Matrix3d &R_ref,
      const Eigen::Vector3d &p_now, const Eigen::Matrix3d &R_now) {
    Eigen::Matrix<double,6,1> err;
    err.head<3>() = p_ref - p_now;
    err.tail<3>() = R_now * rot2omega(R_now.transpose() * R_ref);
    return err;
  }

  // Per-joint state for Jacobian computation
  struct LegFKFull {
    Eigen::Vector3d p_end;
    Eigen::Matrix3d R_end;
    Eigen::Vector3d joint_p[5];  // joint origin in body frame
    Eigen::Vector3d joint_a[5];  // effective joint axis in body frame
  };

  // Forward kinematics tracking joint positions/axes (§2.5.5 Fig. 2.31)
  LegFKFull legFKFull(const Eigen::VectorXd &q, bool right) const {
    const double hx = -0.149, hy = right ? -0.122 : 0.122, hz = -0.103;
    const double L1 = 0.2517, L2 = 0.3472;
    const double afo_x = 0.017, afo_y = right ? -0.023 : 0.023, afo_z = -0.030;
    const double rs = right ? -1.0 : 1.0;  // ankle roll sign convention

    LegFKFull res;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p(hx, hy, hz);

    // Record joint state then apply rotation (Jacobian axis = rotation axis)
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

    // Ankle roll: effective Jacobian axis is rs*UnitX; rotation is rotX(rs*q(4))
    res.joint_p[4] = p;
    res.joint_a[4] = R * (rs * Eigen::Vector3d::UnitX());
    R *= Eigen::AngleAxisd(rs * q(4), Eigen::Vector3d::UnitX()).toRotationMatrix();

    p += R * Eigen::Vector3d(afo_x, afo_y, afo_z);
    res.p_end = p;
    res.R_end = R;
    return res;
  }

  // Analytical Jacobian (§2.5.5 Fig. 2.31): J[:,j] = [a_j × (p_end - p_j); a_j]
  static Eigen::Matrix<double,6,5> calcJacobianKajita(const LegFKFull &fk) {
    Eigen::Matrix<double,6,5> J;
    for (int j = 0; j < 5; ++j) {
      J.block<3,1>(0,j) = fk.joint_a[j].cross(fk.p_end - fk.joint_p[j]);
      J.block<3,1>(3,j) = fk.joint_a[j];
    }
    return J;
  }

  // Levenberg-Marquardt IK with singularity robustness (§2.5.8 Fig. 2.36)
  LegAngles numericalLegIK(double fx, double fy, double fz,
                            double tgt_pitch, double tgt_roll,
                            const LegAngles &guess, bool right) const {
    const Eigen::Vector3d p_ref(fx, fy, fz);
    const Eigen::Matrix3d R_ref =
        Eigen::AngleAxisd(tgt_roll,  Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(tgt_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // Error weights: wn_pos=1/0.3 (m), wn_ang=1/(2π) (rad) — Kajita §2.5.8
    Eigen::Matrix<double,6,1> wv;
    wv << 1.0/0.3, 1.0/0.3, 1.0/0.3, 1.0/(2*M_PI), 1.0/(2*M_PI), 1.0/(2*M_PI);

    Eigen::VectorXd q(5);
    q << guess.hip_roll, guess.hip_pitch, guess.knee_pitch, guess.ankle_pitch,
         right ? -guess.ankle_roll : guess.ankle_roll;

    LegFKFull fk = legFKFull(q, right);
    Eigen::Matrix<double,6,1> err = calcVWerr(p_ref, R_ref, fk.p_end, fk.R_end);
    double Ek = (wv.array() * err.array().square()).sum();

    for (int iter = 0; iter < 10; ++iter) {
      auto J = calcJacobianKajita(fk);
      double lambda = Ek + 0.002;  // adaptive λ (Sugihara's method)
      // Hk = J'*We*J + λ*I,  gk = J'*We*err  (eq. 2.85)
      Eigen::Matrix<double,5,5> Hk =
          J.transpose() * (wv.asDiagonal() * J) +
          lambda * Eigen::Matrix<double,5,5>::Identity();
      Eigen::Matrix<double,5,1> gk = J.transpose() * (wv.asDiagonal() * err);
      Eigen::VectorXd dq = Hk.ldlt().solve(gk);

      q += dq;
      fk = legFKFull(q, right);
      err = calcVWerr(p_ref, R_ref, fk.p_end, fk.R_end);
      double Ek2 = (wv.array() * err.array().square()).sum();

      if (Ek2 < 1e-12)    break;
      else if (Ek2 < Ek)  Ek = Ek2;
      else { q -= dq; break; }  // revert — error increased
    }

    double ank_roll_cmd = right ? -q(4) : q(4);
    return {0.0, q(0), q(1), q(2), q(3), ank_roll_cmd};
  }

  // ── Geometric IK (Fallback) ─────────────────────────────
  LegAngles pureLegIK(double fx, double fy, double fz, bool right) const {
    const double hx = -0.149, hy = right ? -0.122 : 0.122, hz = -0.103;
    const double afo_x = 0.017, afo_y = right ? -0.023 : 0.023, afo_z = -0.030;
    double ax = fx - afo_x, ay = fy - afo_y, az = fz - afo_z;
    double dx = ax - hx, dy = ay - hy, dz = az - hz;
    const double L1 = 0.2517, L2 = 0.3472;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double ck = std::max(
        -1.0, std::min(1.0, (dist * dist - L1 * L1 - L2 * L2) / (2 * L1 * L2)));
    double knee = std::acos(ck);
    double hip_roll = std::atan2(dy, -dz);
    double hip_pitch = std::atan2(dx, std::sqrt(dy * dy + dz * dz));
    double ank_pitch = -(hip_pitch + knee);
    double ank_roll = -hip_roll;
    double ank_roll_cmd = right ? -ank_roll : ank_roll;
    return {
        0.0, hip_roll, hip_pitch, knee, ank_pitch, ank_roll_cmd};
  }

  // ── Joint names ───────────────────────────────────────────────────────
  static auto jointNames() {
    return std::array<std::string, 12>{
        "hip_pitch_r",   "hip_roll_r",   "thigh_yaw_r",   "knee_pitch_r",
        "ankle_pitch_r", "ankle_roll_r", "hip_pitch_l",   "hip_roll_l",
        "thigh_yaw_l",   "knee_pitch_l", "ankle_pitch_l", "ankle_roll_l"};
  }
  static auto settleAngles() {
    // Crouch pose from backup.cpp + slight backward tilt (hip_roll_r=-0.04)
    // to counteract the ~13mm forward CoP bias seen in every settle run.
    return std::array<double, 12>{-0.551, -0.0, 0.0, -0.651, -0.251, 0.0,
                                  -0.551, -0.0, 0.0, -0.651, -0.251, 0.0};
  }

  // ── URDF callback ─────────────────────────────────────────────────────
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

  // ── CoM timer (2 Hz) ─────────────────────────────────────────────────
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
    try {
      geometry_msgs::msg::PointStamped fp, fp2;
      fp.header.frame_id = "foot_r";
      fp.point = {};
      fp2 = tf_buf_->transform(fp, "pelvis", tf2::durationFromSec(0.05));
      if (!walk_active_) {
        h_foot_ = fp2.point.z;
        RCLCPP_INFO_ONCE(get_logger(), "h_foot measured=%.4f", h_foot_);
      }
      double h = wz / mu - fp2.point.z;
      if (!walk_active_) {
        omega0_ = std::sqrt(9.81 / h);
        com_h_ = h;
      }
      omega0_ok_ = true;
    } catch (...) {
    }
  }

  // ── Walk-world frame conversion ──────────────────────────────────────
  std::pair<double, double> walkToWorld(double wx, double wy) const {
    return {wy, -wx};
  }
  std::pair<double, double> worldToWalk(double wx, double wy) const {
    return {-wy, wx};
  }

  // ── Main control (50 Hz) ──────────────────────────────────────────────
  void onControl() {
    double t = (now() - startup_).seconds();
    double settle_dur = 6.0;

    // Phase A: Force-feedback settle
    if (!settle_ok_) {
      auto angs = settleAngles();
      auto [tf, cx, cy] = totalZMP();
      // Start CoP feedback at 1s (enough time for initial crouch to land)
      if (tf > 100 && t > 1.0) {
        double g = get_parameter("settle_gain").as_double();
        // In pelvis frame, +Y is BACKWARD. So if cy > 0, CoP is backward.
        // To lean forward, we need dorsiflexion (negative ankle pitch).
        settle_adj_ -= g * cy * 0.02;
        settle_adj_ = std::max(-0.1, std::min(0.1, settle_adj_));
      }
      angs[4] += settle_adj_;
      angs[10] += settle_adj_;
      publishAngles(angs, 1.5); // slow horizon like backup.cpp

      if (t > 3.0) {
        static int lc = 0;
        if (++lc % 25 == 0)
          RCLCPP_INFO(get_logger(),
                      "Settle: F=%.0f CoP=(%.3f,%.3f) adj=%.4f pitch=%.3f", tf,
                      cx, cy, settle_adj_, imu_pitch_);
        if (std::abs(cy) < 0.005 && std::abs(imu_pitch_) < 0.1 && tf > 100) {
          settle_ok_ = true;
          RCLCPP_INFO(get_logger(), "Settle OK: F=%.0f CoP=%.3f adj=%.4f", tf,
                      cy, settle_adj_);
        } else if (t > settle_dur) {
          settle_ok_ = true;
          RCLCPP_WARN(get_logger(),
                      "Settle timeout: F=%.0f CoP=%.3f pitch=%.3f", tf, cy,
                      imu_pitch_);
        }
      }
      return;
    }

    if (!omega0_ok_)
      return;

    // Phase B: Init walk
    if (!walk_active_) {
      if (walk_done_)
        return;
      try {
        initFootsteps();
      } catch (...) {
        return;
      }
      double sl = get_parameter("step_length").as_double();
      double tss = get_parameter("t_ss").as_double();
      double wT = omega0_ * tss;
      double xdot0 = sl * omega0_ * sinh(wT) / (2 * (cosh(wT) - 1));
      double xrel = xdot0 * (1 - cosh(wT)) / (omega0_ * sinh(wT));
      double y0 = (steps_[0].y + steps_[1].y) * 0.5;
      double yoff = steps_[1].y - y0;
      double ydot0 = yoff * omega0_ * sinh(wT) / (cosh(wT) + 1);
      computeTraj(steps_[0].x + xrel, y0, xdot0, ydot0);
      RCLCPP_INFO(get_logger(),
                  "Walk IC: x0=%.3f xdot0=%.3f y0=%.3f ydot0=%.3f omega0=%.3f",
                  steps_[0].x + xrel, xdot0, y0, ydot0, omega0_);
      walk_active_ = true;
      walk_start_ = now();
      phase_start_ = now();

      // Capture the physical foot positions relative to the pelvis at Settle
      try {
        geometry_msgs::msg::PointStamped orig, pt;
        orig.point.x = orig.point.y = orig.point.z = 0;
        orig.header.frame_id = "foot_r";
        pt = tf_buf_->transform(orig, "pelvis", tf2::durationFromSec(0.05));
        settle_foot_r_x_ = pt.point.x;
        settle_foot_r_y_ = pt.point.y;
        settle_foot_r_z_ = pt.point.z;

        orig.header.frame_id = "foot_l";
        pt = tf_buf_->transform(orig, "pelvis", tf2::durationFromSec(0.05));
        settle_foot_l_x_ = pt.point.x;
        settle_foot_l_y_ = pt.point.y;
        settle_foot_l_z_ = pt.point.z;

        RCLCPP_INFO(get_logger(),
                    "FK Anchors Captured:\n  R: (%.3f, %.3f, %.3f)\n  L: "
                    "(%.3f, %.3f, %.3f)",
                    settle_foot_r_x_, settle_foot_r_y_, settle_foot_r_z_,
                    settle_foot_l_x_, settle_foot_l_y_, settle_foot_l_z_);
      } catch (tf2::TransformException &e) {
        RCLCPP_WARN(get_logger(), "Failed to capture settle foot FK: %s",
                    e.what());
      }

      // Capture Pure IK offsets to guarantee mathematically perfect continuity
      // IK Frame: X=Forward, Y=Left. Pelvis Frame: -Y=Forward, X=Left.
      // So IK_X = -Pelvis_Y, IK_Y = Pelvis_X
      r_settle_pure_ = pureLegIK(-settle_foot_r_y_, settle_foot_r_x_, settle_foot_r_z_, true);
      l_settle_pure_ = pureLegIK(-settle_foot_l_y_, settle_foot_l_x_, settle_foot_l_z_, false);
      
      r_pure_last_ = r_settle_pure_;
      l_pure_last_ = l_settle_pure_;

      phase_ = DS;
      step_ = 0;
      rflag_ = true;
      ds_start_cx_ = steps_[0].x; // Initial walk frame X (fx0)
      ds_start_cy_ = 0.0;         // Initial walk frame Y
      imu_pitch_ = 0;
      imu_roll_ = 0;
      imu_pitch_rate_ = 0;
      imu_roll_rate_ = 0;
      RCLCPP_INFO(get_logger(), "Walk started");
    }

    // Phase C.1: State Machine Transitions
    double pt = (now() - phase_start_).seconds();
    double tss = get_parameter("t_ss").as_double();
    double tds = get_parameter("t_ds").as_double();

    if (phase_ == DS) {
      double dd = (step_ == 0) ? 2 * tds : tds;
      auto [rf, _1, _2] = footCoP(true);
      auto [lf, _3, _4] = footCoP(false);
      double tot = rf + lf;
      if (pt > dd) {
        bool next_is_right = ((step_ + 1) % 2 == 0);
        double sf = next_is_right ? rf : lf;
        bool ok = tot < 10 || sf > 0.4 * tot;
        if (ok || pt > 2 * dd) {
          step_++;
          phase_ = (step_ % 2 == 0) ? SR : SL;
          phase_start_ = now();
          pt = 0.0;
        }
      }
    } else {
      if (pt > tss) {
        if (step_ >= N_ - 2) {
          walk_active_ = false;
          walk_done_ = true;
          RCLCPP_INFO(get_logger(), "Walk finished");
          return;
        }
        bool swing_r = (phase_ == SL);
        auto [sf, _1, _2] = footCoP(swing_r);
        if (sf > 50 || pt > 2 * tss) {
          phase_ = DS;
          phase_start_ = now();
          pt = 0.0;
        }
      }
    }

    // Phase C.2: ZMP and LIPM Trajectory
    double alpha = 0.0;
    if (phase_ == DS) {
      double dd = (step_ == 0) ? 2 * tds : tds;
      alpha = std::min(pt / dd, 1.0);
    } else {
      alpha = std::min(pt / tss, 1.0);
    }
    
    // Smooth dynamic LIPM tracking
    if (!traj_.empty()) {
      int pts = std::max(1, (int)(tss / 0.02));
      int t_idx = 0;
      if (phase_ == DS) {
        t_idx = step_ * pts;
      } else {
        int base = std::max(0, (step_ - 1) * pts);
        int off = (int)(pt / 0.02);
        t_idx = base + off;
      }
      t_idx = std::min(t_idx, (int)traj_.size() - 1);
      zmp_x_ = traj_[t_idx].x;
      zmp_y_ = traj_[t_idx].y;
    } else {
      if (phase_ == DS) {
        if (step_ == 0) {
          zmp_x_ = steps_[0].x + alpha * (steps_[1].x - steps_[0].x);
          zmp_y_ = 0.0 + alpha * (steps_[1].y - 0.0);
        } else if (step_ + 1 < N_) {
          zmp_x_ = steps_[step_].x + alpha * (steps_[step_ + 1].x - steps_[step_].x);
          zmp_y_ = steps_[step_].y + alpha * (steps_[step_ + 1].y - steps_[step_].y);
        }
      } else {
        zmp_x_ = steps_[step_].x;
        zmp_y_ = steps_[step_].y;
      }
    }

    // Phase D: CoM Tracking with Counter-Sway
    // For an ultra-conservative near-static gait, we track ZMP directly.
    // However, the swing leg is extremely heavy. If the pelvis is perfectly 
    // centered over the stance foot, the true CoM is pulled toward the swing leg, 
    // causing a lateral fall. We must over-shift the pelvis by `com_sway`.
    double sway = get_parameter("com_sway").as_double();
    double target_sway = 0.0;
    
    if (phase_ == SR) {
      target_sway = -sway; // Stance is Right, shift pelvis further Right (negative)
    } else if (phase_ == SL) {
      target_sway = sway;  // Stance is Left, shift pelvis further Left (positive)
    } else if (phase_ == DS) {
      double next_sway = ((step_ + 1) % 2 == 0) ? -sway : sway; 
      double prev_sway = (step_ == 0) ? 0.0 : ((step_ % 2 == 0) ? -sway : sway);
      target_sway = prev_sway + alpha * (next_sway - prev_sway);
    }
    
    double cx = zmp_x_;
    double cy = zmp_y_ + target_sway;

    // Phase E: Foot targets in walk frame
    double sfx = steps_[step_].x, sfy = steps_[step_].y;
    int ni = std::min(step_ + 1, N_ - 1); // Next stance step
    int ppi = std::max(0, step_ - 2); // Previous stance foot of the SAME leg
    
    // For single support, the swing foot moves from its previous stance (ppi) to its next stance (ni)
    double sw_start_x = steps_[ppi].x;
    double sw_start_y = steps_[ppi].y;

    double sh = get_parameter("step_height").as_double();
    double swx, swy, swz;
    if (phase_ == DS) {
      swx = steps_[ni].x;
      swy = steps_[ni].y;
      swz = h_foot_;
    } else {
      double sm = alpha * alpha * (3 - 2 * alpha);
      swx = sw_start_x + sm * (steps_[ni].x - sw_start_x);
      swy = sw_start_y + sm * (steps_[ni].y - sw_start_y);
      swz = h_foot_ + sh * std::sin(M_PI * alpha);
    }

    // Phase F: Calculate Delta from Trajectory and Apply to FK Anchors
    // Walk frame: sfx, cx are Forward. sfy, cy are Lateral.
    double r_walk_fwd, r_walk_lat, r_z;
    double l_walk_fwd, l_walk_lat, l_z;

    if (step_ % 2 == 0) { // Right foot is Stance
      r_walk_fwd = sfx - cx;
      r_walk_lat = sfy - cy;
      r_z = h_foot_;
      l_walk_fwd = swx - cx;
      l_walk_lat = swy - cy;
      l_z = swz;
    } else { // Left foot is Stance
      l_walk_fwd = sfx - cx;
      l_walk_lat = sfy - cy;
      l_z = h_foot_;
      r_walk_fwd = swx - cx;
      r_walk_lat = swy - cy;
      r_z = swz;
    }

    // Subtract the initial offset at t=0 to get a PURE DELTA
    // At t=0, CoM was at cx0, 0
    double cx0 = steps_[0].x;
    double cy0 = 0.0;

    double r_init_fwd = steps_[0].x - cx0;
    double r_init_lat = steps_[0].y - cy0;
    double l_init_fwd = steps_[1].x - cx0;
    double l_init_lat = steps_[1].y - cy0;

    double r_delta_fwd = r_walk_fwd - r_init_fwd;
    double r_delta_lat = r_walk_lat - r_init_lat;
    double l_delta_fwd = l_walk_fwd - l_init_fwd;
    double l_delta_lat = l_walk_lat - l_init_lat;

    // Apply pure delta to the recorded Settle FK anchors
    // IK Frame and Walk Frame are aligned (X=Forward, Y=Left)
    // Pelvis anchor to IK anchor: IK_X = -Pelvis_Y, IK_Y = Pelvis_X
    double r_ik_fwd = -settle_foot_r_y_;
    double r_ik_lat = settle_foot_r_x_;
    double l_ik_fwd = -settle_foot_l_y_;
    double l_ik_lat = settle_foot_l_x_;

    double tgt_r_fwd = r_ik_fwd + r_delta_fwd;
    double tgt_r_lat = r_ik_lat + r_delta_lat;
    double tgt_r_z = r_z;

    double tgt_l_fwd = l_ik_fwd + l_delta_fwd;
    double tgt_l_lat = l_ik_lat + l_delta_lat;
    double tgt_l_z = l_z;

    // Use Newton-Raphson Numerical IK
    r_pure_last_ = numericalLegIK(tgt_r_fwd, tgt_r_lat, tgt_r_z, 0.0, 0.0, r_pure_last_, true);
    l_pure_last_ = numericalLegIK(tgt_l_fwd, tgt_l_lat, tgt_l_z, 0.0, 0.0, l_pure_last_, false);

    auto scmd = settleAngles();
    std::array<double, 12> j;
    
    // Apply pure kinematic deltas to Settle Motor Commands
    // Right Leg
    j[0] = scmd[0] - (r_pure_last_.hip_pitch - r_settle_pure_.hip_pitch);
    j[1] = scmd[1] + (r_pure_last_.hip_roll - r_settle_pure_.hip_roll);
    j[2] = scmd[2];
    j[3] = scmd[3] - (r_pure_last_.knee_pitch - r_settle_pure_.knee_pitch);
    j[4] = scmd[4] + (r_pure_last_.ankle_pitch - r_settle_pure_.ankle_pitch);
    j[5] = scmd[5] + (r_pure_last_.ankle_roll - r_settle_pure_.ankle_roll);

    // Left Leg
    j[6] = scmd[6] - (l_pure_last_.hip_pitch - l_settle_pure_.hip_pitch);
    j[7] = scmd[7] + (l_pure_last_.hip_roll - l_settle_pure_.hip_roll);
    j[8] = scmd[8];
    j[9] = scmd[9] - (l_pure_last_.knee_pitch - l_settle_pure_.knee_pitch);
    j[10] = scmd[10] + (l_pure_last_.ankle_pitch - l_settle_pure_.ankle_pitch);
    j[11] = scmd[11] + (l_pure_last_.ankle_roll - l_settle_pure_.ankle_roll);

    // Phase G: Ankle feedback (all gains 0 by default — tune live once LIPM
    // stable)
    {
      double kc = get_parameter("kp_cop").as_double();
      if (kc > 0.0) {
        auto [rf, _1, rcy] = footCoP(true);
        auto [lf, _2, lcy] = footCoP(false);
        if (rf > 5)
          j[4] -= kc * rcy;
        if (lf > 5)
          j[10] -= kc * lcy;
      }

      double kp = get_parameter("kp_pitch").as_double();
      if (kp > 0.0 && imu_ok_) {
        double kd = get_parameter("kd_pitch").as_double();
        double mc = get_parameter("max_corr").as_double();
        // pitch>0=leaning back → dorsiflexion (negative ankle)
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
        j[11] -= rc; // opposite sign for left ankle roll
      }
    }

    // Log
    static rclcpp::Time ll = now();
    if ((now() - ll).seconds() > 0.5) {
      ll = now();
      const char *ps = phase_ == DS ? "DS" : phase_ == SL ? "SL" : "SR";
      RCLCPP_INFO(get_logger(),
                  "[%d|%s|a=%.2f] cx=%.3f cy=%.3f\n"
                  "  R foot(%.3f,%.3f,%.3f) IK: hp=%.3f hr=%.3f kn=%.3f "
                  "ap=%.3f ar=%.3f\n"
                  "  L foot(%.3f,%.3f,%.3f) IK: hp=%.3f hr=%.3f kn=%.3f "
                  "ap=%.3f ar=%.3f\n"
                  "  IMU: pitch=%.3f roll=%.3f",
                  step_, ps, alpha, cx, cy, tgt_r_fwd, tgt_r_lat, tgt_r_z,
                  r_pure_last_.hip_roll, r_pure_last_.hip_pitch, r_pure_last_.knee_pitch, r_pure_last_.ankle_pitch,
                  r_pure_last_.ankle_roll, tgt_l_fwd, tgt_l_lat, tgt_l_z, l_pure_last_.hip_roll,
                  l_pure_last_.hip_pitch, l_pure_last_.knee_pitch, l_pure_last_.ankle_pitch, l_pure_last_.ankle_roll,
                  imu_pitch_, imu_roll_);
    }

    // Publish — fixed short horizon so joints track LIPM reference tightly
    publishAngles(j, 0.08);
    publishMarkers(cx, cy);
  }

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

  void publishMarkers(double cx, double cy) {
    visualization_msgs::msg::MarkerArray arr;
    auto now_t = now();
    auto mk = [&](int id, int type, float r, float g, float b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
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
    // Desired CoM — blue sphere
    {
      auto m = mk(0, 2, 0, 0.4f, 1.0f);
      auto [wx, wy] = walkToWorld(cx, cy);
      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = com_h_;
      m.scale.x = m.scale.y = m.scale.z = 0.06;
      arr.markers.push_back(m);
    }
    // Trajectory — green line
    {
      auto m = mk(1, 4, 0, 1.0f, 0);
      m.scale.x = 0.01;
      for (auto &tp : traj_) {
        geometry_msgs::msg::Point p;
        auto [wx, wy] = walkToWorld(tp.x, tp.y);
        p.x = wx;
        p.y = wy;
        p.z = com_h_;
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }
    // ZMP — red sphere
    {
      auto m = mk(2, 2, 1.0f, 0.1f, 0);
      auto [wx, wy] = walkToWorld(zmp_x_, zmp_y_);
      m.pose.position.x = wx;
      m.pose.position.y = wy;
      m.pose.position.z = 0.02;
      m.scale.x = m.scale.y = m.scale.z = 0.05;
      arr.markers.push_back(m);
    }
    // Footsteps — right=orange, left=cyan
    for (int i = 0; i < (int)steps_.size(); i++) {
      float fr = (i % 2 == 0) ? 1.0f : 0.0f, fg = (i % 2 == 0) ? 0.5f : 0.9f,
            fb = (i % 2 == 0) ? 0.0f : 0.9f;
      auto m = mk(10 + i, 1, fr, fg, fb);
      auto [wx, wy] = walkToWorld(steps_[i].x, steps_[i].y);
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

  void initFootsteps() {
    auto fr = tf_buf_->lookupTransform("world", "foot_r", tf2::TimePointZero);
    auto fl = tf_buf_->lookupTransform("world", "foot_l", tf2::TimePointZero);
    auto [frx, fry] =
        worldToWalk(fr.transform.translation.x, fr.transform.translation.y);
    auto [flx, fly] =
        worldToWalk(fl.transform.translation.x, fl.transform.translation.y);
    double fx0 = (frx + flx) * 0.5;
    double sep = (fly - fry) * 0.5; // Dynamically match exact physical stance width
    double ry = fx0 - sep,
           ly = fx0 + sep; // wrong variable name but matches backup logic
    // Actually: foot_r_y = fy_center - sep, foot_l_y = fy_center + sep
    double fy0 = (fry + fly) * 0.5;
    double fr_y = fy0 - sep, fl_y = fy0 + sep;
    double sl = get_parameter("step_length").as_double();
    steps_.clear();
    for (int i = 0; i < N_; i++) {
      double x = (i <= 1) ? fx0 : fx0 + (i - 1) * sl;
      steps_.push_back({x, (i % 2 == 0) ? fr_y : fl_y});
    }
    RCLCPP_INFO(get_logger(), "Footsteps: fx0=%.3f fr_y=%.3f fl_y=%.3f", fx0,
                fr_y, fl_y);
  }

  void computeTraj(double x0, double y0, double xdot0, double ydot0) {
    traj_.clear();
    double tss = get_parameter("t_ss").as_double();
    double sl = get_parameter("step_length").as_double();
    
    double x_rel = xdot0 * (1 - cosh(omega0_ * tss)) / (omega0_ * sinh(omega0_ * tss));
    
    // Calculate global time
    double t_global = 0.0;
    
    for (int i = 0; i + 1 < N_; i++) {
      double zx = steps_[i + 1].x, zy = steps_[i + 1].y;
      
      // Exact analytical start state for this step to prevent exponential explosion
      double x_start = zx + x_rel;
      double xd_start = xdot0;
      
      // For Y, the start state is always perfectly symmetric around y0
      double y_start = y0;
      double yd_start = (i % 2 == 0) ? ydot0 : -ydot0;
      
      for (double t = 0; t < tss - 1e-9; t += 0.02) {
        double c = cosh(omega0_ * t);
        double s = sinh(omega0_ * t);
        
        double xn = (x_start - zx) * c + (xd_start / omega0_) * s + zx;
        double xdn = (x_start - zx) * omega0_ * s + xd_start * c;
        
        double yn = (y_start - zy) * c + (yd_start / omega0_) * s + zy;
        double ydn = (y_start - zy) * omega0_ * s + yd_start * c;
        
        traj_.push_back({t_global, xn, xdn, yn, ydn});
        t_global += 0.02;
      }
    }
    RCLCPP_INFO(get_logger(),
                "Traj: %zu pts  IC: x=%.3f xd=%.3f y=%.3f yd=%.3f",
                traj_.size(), x0, xdot0, y0, ydot0);
  }

  // ── Members ───────────────────────────────────────────────────────────
  enum Phase { DS, SL, SR };
  static constexpr int N_ = 8;

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
  double total_mass_{0}, omega0_{0}, com_h_{0.7}, com_x_{0}, com_y_{0};
  double h_foot_{-0.659};
  bool urdf_ok_{false}, omega0_ok_{false};

  double imu_pitch_{0}, imu_roll_{0}, imu_pitch_rate_{0}, imu_roll_rate_{0};
  bool imu_ok_{false};
  ForceSensor fsens_[8];

  bool settle_ok_{false};
  double settle_adj_{0};
  bool walk_active_{false}, walk_done_{false};
  rclcpp::Time walk_start_, phase_start_;
  Phase phase_{DS};
  int step_{0};
  bool rflag_{true};
  double zmp_x_{0}, zmp_y_{0};
  std::vector<Footstep> steps_;
  std::vector<TrajPoint> traj_;

  double ds_start_cx_{0}, ds_start_cy_{0};
  
  LegAngles r_settle_pure_, l_settle_pure_;
  LegAngles r_pure_last_, l_pure_last_; // For Newton Raphson guess

  // Settle position memory (FK of feet in pelvis frame at t=0)
  double settle_foot_r_x_{0}, settle_foot_r_y_{0}, settle_foot_r_z_{0};
  double settle_foot_l_x_{0}, settle_foot_l_y_{0}, settle_foot_l_z_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZmpWalker>());
  rclcpp::shutdown();
}
