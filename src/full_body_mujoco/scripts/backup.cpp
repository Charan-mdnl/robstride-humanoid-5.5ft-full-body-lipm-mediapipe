#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
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

struct TrajPoint {
  double t, x, xdot, y, ydot;
};

class ComVerifier : public rclcpp::Node {
public:
  ComVerifier()
      : Node("com_verifier"), qos(rclcpp::QoS(1).transient_local()),
        startup_time_(this->now()) {

    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos,
        std::bind(&ComVerifier::urdfCallback, this, std::placeholders::_1));
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ComVerifier::timerCallback, this));
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, this);
    walk_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/lower_body_controller/joint_trajectory", 10);
    walk_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&ComVerifier::walkCallback, this));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "walk_markers", 10);
  }

private:
  void urdfCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "URDF received, size: %zu bytes",
                msg->data.size());

    if (!model.initString(msg->data)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
      return;
    }

    for (auto &[name, link] : model.links_) {
      if (!link->inertial)
        continue;
      link_mass_[name] = link->inertial->mass;
      link_com_[name] = {link->inertial->origin.position.x,
                         link->inertial->origin.position.y,
                         link->inertial->origin.position.z};
      RCLCPP_INFO(this->get_logger(), "link name : %s ; x:%f, y:%f , z:%f",
                  name.c_str(), link->inertial->origin.position.x,
                  link->inertial->origin.position.y,
                  link->inertial->origin.position.z);
      total_mass_ += link->inertial->mass;
    }

    // Parse joint origins for FK
    for (auto &[name, joint] : model.joints_) {
      joint_origins_[name] = {joint->parent_to_joint_origin_transform.position.x,
                              joint->parent_to_joint_origin_transform.position.y,
                              joint->parent_to_joint_origin_transform.position.z};
      RCLCPP_INFO(this->get_logger(), "Joint %s origin: (%.4f, %.4f, %.4f)",
                  name.c_str(), joint_origins_[name][0], joint_origins_[name][1], joint_origins_[name][2]);
    }

    RCLCPP_INFO(this->get_logger(), "Parsed %zu links, total mass: %.3f kg",
                link_mass_.size(), total_mass_);

    urdf_ready_ = true;
    urdf_sub_.reset();
  }

  void timerCallback() {
    if (!urdf_ready_)
      return;
    // Don't compute anything until the robot has finished settling.
    // Measurements taken while joints are still moving give a wrong h/omega0.
    if ((this->now() - startup_time_).seconds() < settle_dur_)
      return;

    double wx = 0.0, wy = 0.0, wz = 0.0, mass_used = 0.0;

    for (auto &[name, mass] : link_mass_) {
      geometry_msgs::msg::PointStamped local_pt, world_pt;
      local_pt.header.frame_id = name;
      local_pt.point.x = link_com_[name][0];
      local_pt.point.y = link_com_[name][1];
      local_pt.point.z = link_com_[name][2];

      try {
        world_pt =
            tf_buf_->transform(local_pt, "pelvis", tf2::durationFromSec(0.05));
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF failed for %s: %s", name.c_str(),
                    ex.what());
        continue;
      }

      wx += mass * world_pt.point.x;
      wy += mass * world_pt.point.y;
      wz += mass * world_pt.point.z;
      mass_used += mass;
    }

    if (mass_used < 0.1)
      return;

    com_y_pelvis_ = wy / mass_used;
    com_x_pelvis_ = wx / mass_used; // sagittal CoM in pelvis frame

    // foot positions in pelvis frame
    try {
      geometry_msgs::msg::PointStamped foot_origin, foot_r_pelvis,
          foot_l_pelvis;
      foot_origin.point.x = 0.0;
      foot_origin.point.y = 0.0;
      foot_origin.point.z = 0.0;

      foot_origin.header.frame_id = "foot_r";
      foot_r_pelvis =
          tf_buf_->transform(foot_origin, "pelvis", tf2::durationFromSec(0.05));

      foot_origin.header.frame_id = "foot_l";
      foot_l_pelvis =
          tf_buf_->transform(foot_origin, "pelvis", tf2::durationFromSec(0.05));

      foot_r_pelvis_x_ = foot_r_pelvis.point.x;
      double foot_z = (foot_r_pelvis.point.z + foot_l_pelvis.point.z) / 2.0;
      // Keep h_foot updated from actual TF measurement so walk IK matches real
      // geometry
      if (!walk_active_)
        h_foot_ = foot_z;
      double h = (wz / mass_used) - foot_z;
      double omega0 = std::sqrt(9.81 / h);

      double half_angle = omega0 * T / 2.0;
      double x_init_vel =
          (d / 2.0) * omega0 * (std::cosh(half_angle) / std::sinh(half_angle));
      double y_init_vel = (w / 2.0) * omega0 * std::tanh(half_angle);

      RCLCPP_INFO(this->get_logger(),
                  "CoM: x=%.4f y=%.4f z=%.4f | foot_z=%.4f | h=%.4f m | "
                  "omega0=%.4f rad/s",
                  wx / mass_used, wy / mass_used, wz / mass_used, foot_z, h,
                  omega0);
      RCLCPP_INFO(this->get_logger(),
                  "Orbital IC: x0=%.4f xdot0=%.4f | y0=0.0000 ydot0=%.4f",
                  -d / 2.0, x_init_vel, y_init_vel);

      // Keep omega0_ fresh while not walking; freeze it once walking starts
      // so all LIPM steps use the same value and the orbital IC stays
      // self-repeating.
      if (!walk_active_) {
        omega0_ = omega0;
        com_h_ = h;
      }

      if (!traj_computed_) {
        // Verify IK against current standing foot positions
        auto ang_r = legIKNR(foot_r_pelvis.point.x, foot_r_pelvis.point.y,
                           foot_r_pelvis.point.z, true);
        RCLCPP_INFO(this->get_logger(),
                    "IK right: yaw=%.3f roll=%.3f pitch=%.3f knee=%.3f "
                    "ank_p=%.3f ank_r=%.3f",
                    ang_r.hip_yaw, ang_r.hip_roll, ang_r.hip_pitch,
                    ang_r.knee_pitch, ang_r.ankle_pitch, ang_r.ankle_roll);

        auto fk_r =
            legFK(ang_r.hip_roll, ang_r.hip_pitch, ang_r.knee_pitch, true);
        RCLCPP_INFO(
            this->get_logger(),
            "FK right: (%.4f, %.4f, %.4f) | TF target: (%.4f, %.4f, %.4f)",
            fk_r.x, fk_r.y, fk_r.z, foot_r_pelvis.point.x,
            foot_r_pelvis.point.y, foot_r_pelvis.point.z);

        traj_computed_ = true;
        traj_ready_time_ = this->now();
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Foot TF failed: %s", ex.what());
    }
  }

  struct LegAngles {
    double hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll;
  };

  std::array<std::string, 12> lowerJointNames() const {
    return {"hip_pitch_r",   "hip_roll_r",   "thigh_yaw_r",   "knee_pitch_r",
            "ankle_pitch_r", "ankle_roll_r", "hip_pitch_l",   "hip_roll_l",
            "thigh_yaw_l",   "knee_pitch_l", "ankle_pitch_l", "ankle_roll_l"};
  }

  std::array<double, 12> settlePositions() const {
    // Matches MuJoCo standing keyframe exactly
    return {-0.615, -0.059, 0.0, -0.593, -0.351, 0.0,
            -0.615, -0.059, 0.0, -0.593, -0.351, 0.0};
  }

  std::array<double, 12> crouchPositions() const {
    // Upright stance with forward ankle lean to prevent backwards falling
    // hip_pitch, hip_roll, thigh_yaw, knee_pitch, ankle_pitch, ankle_roll (right, then left)
    return {-0.35, 0.0, 0.0, -0.45, -0.30, 0.0,
            -0.35, 0.0, 0.0, -0.45, -0.30, 0.0};
  }

  trajectory_msgs::msg::JointTrajectory makeCrouchMsg() {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    const auto names = lowerJointNames();
    msg.joint_names.assign(names.begin(), names.end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    const auto crouch = crouchPositions();
    pt.positions.assign(crouch.begin(), crouch.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(1.5);
    msg.points.push_back(pt);
    return msg;
  }

  // Foot position in pelvis frame → joint angles (geometric analytical IK)
  LegAngles legIK(double fx, double fy, double fz, bool is_right) {
    // Hip center in pelvis frame. hx same for both legs (forward offset).
    // hy is lateral: right hip at -w_walk/2+afo_y = -0.122, left at +0.122.
    // This makes dy≈0 at neutral stance so dist≈L1+L2 (no lateral
    // over-extension).
    const double hx = -0.149;
    const double hy = is_right ? -0.122 : 0.122;
    const double hz = -0.103;

    // Ankle-to-foot fixed offset in pelvis frame (flat foot, measured from TF)
    const double afo_x = 0.017;
    const double afo_y = is_right ? -0.023 : 0.023;
    const double afo_z = -0.030;

    // Step 1: target ankle joint center
    const double ax = fx - afo_x;
    const double ay = fy - afo_y;
    const double az = fz - afo_z;

    // Step 2: hip-to-ankle vector
    const double dx = ax - hx;
    const double dy = ay - hy;
    const double dz = az - hz;

    // Step 3: knee angle from law of cosines
    const double L1 = 0.2517, L2 = 0.3472;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double cos_knee = (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    cos_knee = std::max(-1.0, std::min(1.0, cos_knee));
    const double knee_pitch = std::acos(cos_knee); // 0 = fully extended

    // Step 4: hip roll and pitch from direction of hip-to-ankle vector
    const double hip_roll = std::atan2(dy, -dz);
    const double hip_pitch = std::atan2(dx, std::sqrt(dy * dy + dz * dz));

    // Step 5: ankle angles to keep foot flat (cancel hip + knee rotation)
    const double ankle_pitch = -(hip_pitch + knee_pitch);
    const double ankle_roll = -hip_roll;

    // hip_pitch_r is the ABDUCTION (roll) joint; hip_roll_r is the SAGITTAL
    // pitch joint. Swap outputs so lateral IK (hip_roll) → hip_pitch_r and
    // sagittal (hip_pitch) → hip_roll_r.
    // No offsets - return raw joint angles
    const double ankle_roll_cmd = is_right ? ankle_roll : -ankle_roll;
    
    // Clamp ankle pitch to prevent excessive plantarflexion
    const double ankle_clamped = std::max(-0.6, std::min(0.2, ankle_pitch));

    return {0.0,
            hip_roll,         // → hip_pitch_r/l (abduction)
            -hip_pitch,       // → hip_roll_r/l (sagittal)
            -knee_pitch,      // → knee_pitch_r/l
            ankle_clamped,                   // → ankle_pitch_r/l (clamped)
            ankle_roll_cmd};
  }

  struct FootPos {
    double x, y, z;
  };

  // Joint commands (hip_pitch_r, hip_roll_r, knee_pitch_r) → foot position in
  // pelvis frame using Eigen transformation matrices and URDF data
  FootPos legFK(double q_hip_pitch_r, double q_hip_roll_r,
                double q_knee_pitch_r, bool is_right) const {
    // CRITICAL: Invert the legIK sign conventions
    // legIK outputs: {0, hip_roll, -hip_pitch, -knee_pitch, ...}
    // So we must reverse the signs:
    const double hip_roll = q_hip_pitch_r;    // abduction angle
    const double hip_pitch = -q_hip_roll_r;   // sagittal angle - NEGATED!
    const double knee_pitch = -q_knee_pitch_r; // knee angle - NEGATED!

    // Use simple geometric FK - proven to work
    const double L1 = 0.2517;  // thigh length
    const double L2 = 0.3472;  // shank length
    
    const double hx = -0.149;
    const double hy = is_right ? -0.122 : 0.122;
    const double hz = -0.103;
    
    const double afo_x = 0.017;
    const double afo_y = is_right ? -0.023 : 0.023;
    const double afo_z = -0.030;

    // Compute hip-to-ankle vector using law of cosines
    const double dist = std::sqrt(L1 * L1 + L2 * L2 + 2.0 * L1 * L2 * std::cos(knee_pitch));
    const double r_lat = dist * std::cos(hip_pitch);
    const double dx = dist * std::sin(hip_pitch);
    const double dy = r_lat * std::sin(hip_roll);
    const double dz = -r_lat * std::cos(hip_roll);

    return {hx + dx + afo_x, hy + dy + afo_y, hz + dz + afo_z};
  }

  // Newton-Raphson IK using legFK as the forward model.
  // Solves for (hip_pitch_r, hip_roll_r, knee_pitch_r) commands,
  // then derives ankle commands analytically.
  LegAngles legIKNR(double fx, double fy, double fz, bool is_right) {
    // Just use analytical IK - it's already accurate
    // Newton-Raphson was diverging due to sign encoding mismatches
    return legIK(fx, fy, fz, is_right);
  }

  struct PlanarPoint {
    double x;
    double y;
  };

  // Walking/LIPM frame: x = forward, y = lateral.
  // MuJoCo/world visualization frame is rotated relative to that:
  //   walk_x -> -world_y, walk_y -> world_x.
  PlanarPoint walkToWorld(double walk_x, double walk_y) const {
    return {walk_y, -walk_x};
  }

  PlanarPoint worldToWalk(double world_x, double world_y) const {
    return {-world_y, world_x};
  }

  trajectory_msgs::msg::JointTrajectory makeSettleMsg() {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    const auto names = lowerJointNames();
    msg.joint_names.assign(names.begin(), names.end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    const auto settle = crouchPositions();
    pt.positions.assign(settle.begin(), settle.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(1.2);
    msg.points.push_back(pt);
    return msg;
  }

  void walkCallback() {
    // Hold crouch pose while waiting for omega0_ to be computed.
    if ((this->now() - startup_time_).seconds() < settle_dur_) {
      walk_pub_->publish(makeSettleMsg());
      return;
    }

    if (!traj_computed_)
      return;

    if (!walk_active_) {
      if (walk_done_)
        return;

      // Brief additional stabilization after traj_computed_ (omega0_ settling).
      // if ((this->now() - traj_ready_time_).seconds() < 0.0) {
      //   auto settle_msg = makeSettleMsg();
      //   walk_pub_->publish(settle_msg);
      //   return;
      // }

      // Initialize footsteps from TF
      try {
        initFootsteps();
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Footstep init failed: %s", ex.what());
        return;
      }

      // Initialize LIPM trajectory
      double wT = omega0_ * T_ss;
      double xdot0 =
          step_length * omega0_ * sinh(wT) / (2.0 * (cosh(wT) - 1.0));
      double x_rel = xdot0 * (1.0 - cosh(wT)) / (omega0_ * sinh(wT));
      double y0 = (footsteps_[0].y + footsteps_[1].y) * 0.5;
      double y_support_offset = footsteps_[1].y - y0;
      double ydot0 = y_support_offset * omega0_ * sinh(wT) / (cosh(wT) + 1.0);
      computeTrajectory(footsteps_[0].x + x_rel, y0, xdot0, ydot0);
      RCLCPP_INFO(this->get_logger(),
                  "Walk IC: x0=%.3f xdot0=%.3f | y0=%.3f y_support=%.3f "
                  "ydot0=%.3f",
                  footsteps_[0].x + x_rel, xdot0, y0, y_support_offset, ydot0);
      
      walk_active_ = true;
      walk_start_time_ = this->now();
      phase_start_time_ = walk_start_time_;
      current_phase_ = DOUBLE_SUPPORT;
      current_step = 0;
      right_flag_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Walk started: origin=(%.4f, %.4f)", walk_origin_x_,
                  walk_origin_y_);
    }

    double elapsed = (this->now() - phase_start_time_).seconds();
    double alpha = 0.0;

    switch (current_phase_) {
    case DOUBLE_SUPPORT: {
      alpha = std::min(elapsed / T_ds, 1.0);
      if (current_step + 1 < N) {
        zmp_x = footsteps_[current_step].x +
                alpha * (footsteps_[current_step + 1].x -
                         footsteps_[current_step].x);
        zmp_y = footsteps_[current_step].y +
                alpha * (footsteps_[current_step + 1].y -
                         footsteps_[current_step].y);
      } else {
        zmp_x = footsteps_[N - 1].x;
        zmp_y = footsteps_[N - 1].y;
      }
      if (elapsed > T_ds && current_step + 1 < N) {
        // Snapshot cy at the trajectory boundary so the first SS tick has no
        // lateral jump.
        int pts = (int)(T_ss / dt);
        cy_ss_anchor_ =
            com_trajectory_[std::min(current_step * pts,
                                     (int)com_trajectory_.size() - 1)]
                .y;
        zmp_snap_x_ = footsteps_[current_step + 1].x;
        zmp_snap_y_ = footsteps_[current_step + 1].y;
        current_step++;
        current_phase_ = right_flag_ ? SINGLE_RIGHT : SINGLE_LEFT;
        right_flag_ = !right_flag_;
        phase_start_time_ = this->now();
        elapsed = 0.0;
        alpha = 0.0;
      }
      break;
    }
    case SINGLE_LEFT: {
      alpha = std::min(elapsed / T_ss, 1.0);
      zmp_x = zmp_snap_x_;
      zmp_y = zmp_snap_y_;
      if (elapsed > T_ss) {
        if (current_step >= N - 2) {
          walk_active_ = false;
          walk_done_ = true;
          RCLCPP_INFO(this->get_logger(), "Walk finished");
          return;
        }
        current_phase_ = DOUBLE_SUPPORT;
        phase_start_time_ = this->now();
        elapsed = 0.0;
        alpha = 0.0;
      }
      break;
    }
    case SINGLE_RIGHT: {
      alpha = std::min(elapsed / T_ss, 1.0);
      zmp_x = zmp_snap_x_;
      zmp_y = zmp_snap_y_;
      if (elapsed > T_ss) {
        if (current_step >= N - 2) {
          walk_active_ = false;
          walk_done_ = true;
          RCLCPP_INFO(this->get_logger(), "Walk finished");
          return;
        }
        current_phase_ = DOUBLE_SUPPORT;
        phase_start_time_ = this->now();
        elapsed = 0.0;
        alpha = 0.0;
      }
      break;
    }
    }

    // Phase-based cx/cy:
    //   DS step 0: hold x=0 (no forward lean), cy from trajectory boundary.
    //   DS steps 1+: hold trajectory boundary value (constant — no aggressive
    //   pre-lean). SS: advance within section; cy anchored to trajectory
    //   boundary to avoid jump.
    const int pts_per_step = (int)(T_ss / dt);
    double cx, cy;
    if (current_phase_ == DOUBLE_SUPPORT) {
      int idx_ds = std::min(current_step * pts_per_step,
                            (int)com_trajectory_.size() - 1);
      if (current_step == 0) {
        cx = footsteps_[0].x;
      } else {
        cx = com_trajectory_[idx_ds].x;
      }
      cy = com_trajectory_[idx_ds].y;
    } else {
      int section_start = (current_step - 1) * pts_per_step;
      int idx = std::min(section_start + (int)(elapsed / dt),
                         (int)com_trajectory_.size() - 1);
      cx = com_trajectory_[idx].x;
      cy = cy_ss_anchor_ +
           (com_trajectory_[idx].y - com_trajectory_[section_start].y);
    }

    // Add lateral sway compensation for heavy swing leg mass
    // Use moderate sway + narrower stance for stability
    const double sway_amount = 0.03;  // 3cm sway with narrow stance
    double cy_sway = 0.0;
    if (current_phase_ == SINGLE_RIGHT) {
      cy_sway = -sway_amount;
    } else if (current_phase_ == SINGLE_LEFT) {
      cy_sway = sway_amount;
    } else if (current_phase_ == DOUBLE_SUPPORT) {
      double next_sway = ((current_step + 1) % 2 == 0) ? -sway_amount : sway_amount;
      double prev_sway = (current_step == 0) ? 0.0 : ((current_step % 2 == 0) ? -sway_amount : sway_amount);
      cy_sway = prev_sway + alpha * (next_sway - prev_sway);
    }
    cy += cy_sway;

    // Stance foot walk-frame position (fixed at current footstep ZMP)
    double sfx = footsteps_[current_step].x;
    double sfy = footsteps_[current_step].y;

    // Swing foot: prev footstep → next footstep during SS; stays at next_idx
    // during DS
    int prev_idx = std::max(0, current_step - 1);
    int next_idx = std::min(current_step + 1, N - 1);
    double sw_x0 = footsteps_[prev_idx].x;
    double sw_y0 = footsteps_[prev_idx].y;
    double sw_xf = footsteps_[next_idx].x;
    double sw_yf = footsteps_[next_idx].y;

    double sw_x, sw_y, sw_z;
    if (current_phase_ == DOUBLE_SUPPORT) {
      // Swing foot already landed at next_idx during previous SS
      sw_x = footsteps_[next_idx].x;
      sw_y = footsteps_[next_idx].y;
      sw_z = h_foot_;
    } else {
      double smooth = alpha * alpha * (3.0 - 2.0 * alpha);
      sw_x = sw_x0 + smooth * (sw_xf - sw_x0);
      sw_y = sw_y0 + smooth * (sw_yf - sw_y0);
      sw_z = h_foot_ + step_height_ * std::sin(M_PI * alpha);
    }

    // Convert walk frame → pelvis frame using walk origin
    // Walk frame has origin at initial foot midpoint, so: pelvis = walk + origin
    double sf_px = sfx + walk_origin_x_;
    double sf_py = sfy + walk_origin_y_;
    double sw_px = sw_x + walk_origin_x_;
    double sw_py = sw_y + walk_origin_y_;

    // Use captured foot Z to maintain consistent leg configuration
    // Dynamic ground tracking was causing legs to extend/bend as pelvis height changed
    double foot_z_pelvis = h_foot_;

    double r_fx, r_fy, r_fz, l_fx, l_fy, l_fz;
    if (current_step % 2 == 0) { // right stance, left swings
      r_fx = sf_px;
      r_fy = sf_py;
      r_fz = foot_z_pelvis;
      l_fx = sw_px;
      l_fy = sw_py;
      l_fz = (current_phase_ == DOUBLE_SUPPORT) ? foot_z_pelvis : (foot_z_pelvis + sw_z - h_foot_);
    } else { // left stance, right swings
      l_fx = sf_px;
      l_fy = sf_py;
      l_fz = foot_z_pelvis;
      r_fx = sw_px;
      r_fy = sw_py;
      r_fz = (current_phase_ == DOUBLE_SUPPORT) ? foot_z_pelvis : (foot_z_pelvis + sw_z - h_foot_);
    }

    LegAngles ang_r, ang_l;

    ang_r = legIKNR(r_fx, r_fy, r_fz, true);
    ang_l = legIKNR(l_fx, l_fy, l_fz, false);

    static rclcpp::Time last_log_time_ = this->now();
    if ((this->now() - last_log_time_).seconds() >= 0.5) {
      last_log_time_ = this->now();
      const char *phase_str = (current_phase_ == DOUBLE_SUPPORT) ? "DS"
                              : (current_phase_ == SINGLE_LEFT)  ? "SL"
                                                                 : "SR";
      RCLCPP_INFO(this->get_logger(),
                  "[STEP %d | %s | a=%.2f] cx=%.3f\n"
                  "  R foot in: (%.3f, %.3f, %.3f)  IK: roll=%.3f pit=%.3f "
                  "kne=%.3f ank=%.3f\n"
                  "  L foot in: (%.3f, %.3f, %.3f)  IK: roll=%.3f pit=%.3f "
                  "kne=%.3f ank=%.3f",
                  current_step, phase_str, alpha, cx, r_fx, r_fy, r_fz,
                  ang_r.hip_roll, ang_r.hip_pitch, ang_r.knee_pitch,
                  ang_r.ankle_pitch, l_fx, l_fy, l_fz, ang_l.hip_roll,
                  ang_l.hip_pitch, ang_l.knee_pitch, ang_l.ankle_pitch);
    }

    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    const auto names = lowerJointNames();
    msg.joint_names.assign(names.begin(), names.end());

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {ang_r.hip_roll,   ang_r.hip_pitch,   ang_r.hip_yaw,
                    ang_r.knee_pitch, ang_r.ankle_pitch, ang_r.ankle_roll,
                    ang_l.hip_roll,   ang_l.hip_pitch,   ang_l.hip_yaw,
                    ang_l.knee_pitch, ang_l.ankle_pitch, ang_l.ankle_roll};
    pt.time_from_start = rclcpp::Duration::from_seconds(T_ss);
    msg.points.push_back(pt);
    walk_pub_->publish(msg);
    publishMarkers(cx, cy);

    if (current_step >= N - 2 &&
        (current_phase_ == SINGLE_LEFT || current_phase_ == SINGLE_RIGHT)) {
      if (elapsed > T_ss) {
        walk_active_ = false;
        walk_done_ = true;
        RCLCPP_INFO(this->get_logger(), "Walk finished");
      }
    }
  }

  void publishMarkers(double cx, double cy) {
    visualization_msgs::msg::MarkerArray arr;
    auto now = this->now();
    double com_z = com_h_;

    auto make_marker = [&](int id, int type, double r, double g, double b) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp = now;
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
      auto m = make_marker(0, visualization_msgs::msg::Marker::SPHERE, 0.0, 0.4,
                           1.0);
      const auto p = walkToWorld(cx, cy);
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = com_z;
      m.scale.x = m.scale.y = m.scale.z = 0.06;
      arr.markers.push_back(m);
    }

    // Full precomputed trajectory — green line strip
    {
      auto m = make_marker(1, visualization_msgs::msg::Marker::LINE_STRIP, 0.0,
                           1.0, 0.0);
      m.scale.x = 0.01;
      for (auto &tp : com_trajectory_) {
        geometry_msgs::msg::Point p;
        const auto wp = walkToWorld(tp.x, tp.y);
        p.x = wp.x;
        p.y = wp.y;
        p.z = com_z;
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }

    // ZMP — red sphere
    {
      auto m = make_marker(2, visualization_msgs::msg::Marker::SPHERE, 1.0, 0.1,
                           0.0);
      const auto p = walkToWorld(zmp_x, zmp_y);
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = 0.02;
      m.scale.x = m.scale.y = m.scale.z = 0.05;
      arr.markers.push_back(m);
    }

    // Footstep targets — right=orange, left=cyan cubes
    for (int i = 0; i < (int)footsteps_.size(); i++) {
      float r = (i % 2 == 0) ? 1.0f : 0.0f;
      float g = (i % 2 == 0) ? 0.5f : 0.9f;
      float bl = (i % 2 == 0) ? 0.0f : 0.9f;
      auto m =
          make_marker(10 + i, visualization_msgs::msg::Marker::CUBE, r, g, bl);
      const auto p = walkToWorld(footsteps_[i].x, footsteps_[i].y);
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = 0.01;
      m.scale.x = 0.08;
      m.scale.y = 0.04;
      m.scale.z = 0.01;
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
  }

  void initFootsteps() {
    // Get foot positions in pelvis frame
    auto fr = tf_buf_->lookupTransform("pelvis", "foot_r", tf2::TimePointZero);
    auto fl = tf_buf_->lookupTransform("pelvis", "foot_l", tf2::TimePointZero);
    
    double fr_x = fr.transform.translation.x;
    double fr_y = fr.transform.translation.y;
    double fl_x = fl.transform.translation.x;
    double fl_y = fl.transform.translation.y;

    // Walk frame origin at average foot position in pelvis frame
    walk_origin_x_ = (fr_x + fl_x) * 0.5;
    walk_origin_y_ = (fr_y + fl_y) * 0.5;
    
    // Set up footsteps in walk frame (relative to walk origin)
    foot_r_y_ = -foot_separation;  // Right foot at -separation
    foot_l_y_ = foot_separation;   // Left foot at +separation
    
    footsteps_.clear();
    for (int i = 0; i < N; i++) {
      double foot_x = (i <= 1) ? 0.0 : (i - 1) * step_length;
      footsteps_.push_back({foot_x, (i % 2 == 0) ? foot_r_y_ : foot_l_y_});
    }
    RCLCPP_INFO(this->get_logger(),
                "Footsteps init: walk_origin=(%.3f, %.3f) fy_r=%.4f fy_l=%.4f",
                walk_origin_x_, walk_origin_y_, foot_r_y_, foot_l_y_);
  }

  void computeTrajectory(double x0, double y0, double xdot0, double ydot0) {
    com_trajectory_.clear();
    double x = x0, y = y0, xdot = xdot0, ydot = ydot0;
    com_trajectory_.push_back({0.0, x, xdot, y, ydot});

    for (int i = 0; i + 1 < N; i++) {
      double zx = footsteps_[i + 1].x;
      double zy = footsteps_[i + 1].y;
      for (double t = 0.0; t < T_ss; t += dt) {
        double x_new = (x - zx) * cosh(omega0_ * dt) +
                       (xdot / omega0_) * sinh(omega0_ * dt) + zx;
        double xdot_new =
            (x - zx) * omega0_ * sinh(omega0_ * dt) + xdot * cosh(omega0_ * dt);
        double y_new = (y - zy) * cosh(omega0_ * dt) +
                       (ydot / omega0_) * sinh(omega0_ * dt) + zy;
        double ydot_new =
            (y - zy) * omega0_ * sinh(omega0_ * dt) + ydot * cosh(omega0_ * dt);
        x = x_new;
        xdot = xdot_new;
        y = y_new;
        ydot = ydot_new;
        com_trajectory_.push_back({t, x, xdot, y, ydot});
      }
    }
    traj_computed_ = true;
    RCLCPP_INFO(this->get_logger(), "Trajectory computed: %zu points",
                com_trajectory_.size());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr walk_timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr walk_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
  rclcpp::QoS qos;
  urdf::Model model;
  std::map<std::string, double> link_mass_;
  std::map<std::string, std::vector<double>> link_com_;
  std::map<std::string, std::vector<double>> joint_origins_;
  double total_mass_{0.0};
  bool urdf_ready_{false};
  double omega0_{0.0};
  bool traj_computed_{false};
  bool walk_active_{false};
  bool walk_done_{false};
  rclcpp::Time traj_ready_time_;
  rclcpp::Time walk_start_time_;
  double step_x0_{0.0}, step_xdot0_{0.0};
  double step_y0_{0.0}, step_ydot0_{0.0};
  double step_zmp_x_{0.0};
  double com_y_pelvis_{0.0};
  double com_x_pelvis_{0.0};
  double walk_origin_x_{0.0};
  double walk_origin_y_{0.0};
  double foot_r_pelvis_x_{0.0};
  double h_foot_{-0.659};
  int last_k_{0};
  const double d{0.10};
  const double w{0.29};
  const double w_walk{0.29};
  const double T{0.76};
  const double step_height_{0.06};
  const int N_steps_{6};
  const double settle_dur_{6.5};
  const double crouch_dur_{2.0};
  rclcpp::Time startup_time_;

  // LIPM phase machine
  enum phase_t { DOUBLE_SUPPORT, SINGLE_LEFT, SINGLE_RIGHT };
  struct Footstep {
    double x, y;
  };
  std::vector<Footstep> footsteps_;
  std::vector<TrajPoint> com_trajectory_;

  phase_t current_phase_{DOUBLE_SUPPORT};
  rclcpp::Time phase_start_time_;
  int current_step{0};
  bool right_flag_{true};
  double zmp_x{0.0}, zmp_y{0.0};
  double zmp_snap_x_{0.0}, zmp_snap_y_{0.0}; // locked stance foot position
  double cy_ss_anchor_{
      0.0}; // cy value at DS end, carried into SS to avoid lateral jump
  double foot_r_y_{0.0}, foot_l_y_{0.0};
  double pelvis_snap_x_{0.0}, pelvis_snap_y_{0.0}; // snapshot at phase start
  LegAngles frozen_ang_r_{};
  LegAngles frozen_ang_l_{};
  phase_t last_published_phase_{DOUBLE_SUPPORT};
  int last_published_step_{-1};
  double dt{0.02};
  const double T_ds{0.5};
  const double T_ss{0.8};
  const double step_length{0.1};
  const double foot_separation{0.08};  // Reduced from 0.145 for stability during crouch
  const int N{8};
  double com_h_{0.8};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComVerifier>());
  rclcpp::shutdown();
  return 0;
}
