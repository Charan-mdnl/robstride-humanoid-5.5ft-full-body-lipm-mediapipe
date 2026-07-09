// nr_ik_test.cpp
// Newton-Raphson IK test: walk the robot in crouch using pure IK, no LIPM.
// FK is built by walking the URDF kinematic chain pelvis→foot_{r,l}.
// NR iteration minimises ||FK(q) - target||² with damped pseudoinverse.
// Stepping: right foot forward → left foot forward, repeating.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace Eigen;

// ─────────────────────────────────────────────────────────────────────────────
// Kinematic chain representation
// ─────────────────────────────────────────────────────────────────────────────
struct ChainJoint {
  std::string name;
  bool is_revolute;
  Isometry3d origin;   // parent-link frame → this joint frame (fixed part)
  Vector3d   axis;     // rotation axis in the joint frame
  int        q_idx;    // index in q vector; -1 for fixed joints
  double     q_min, q_max;
};

// Walk URDF tree from base_link to tip_link, appending joints to chain.
static bool buildChain(const urdf::Model &model,
                       const std::string &base,
                       const std::string &tip,
                       std::vector<ChainJoint> &chain,
                       int &q_count) {
  if (base == tip) return true;

  auto link = model.getLink(tip);
  if (!link || !link->parent_joint) return false;

  auto j = link->parent_joint;
  std::string parent = j->parent_link_name;

  // Recurse up to base first, so joints come in parent→child order
  if (!buildChain(model, base, parent, chain, q_count)) return false;

  ChainJoint cj;
  cj.name = j->name;
  cj.is_revolute = (j->type == urdf::Joint::REVOLUTE ||
                    j->type == urdf::Joint::CONTINUOUS);

  // Origin transform
  auto &p = j->parent_to_joint_origin_transform.position;
  auto &r = j->parent_to_joint_origin_transform.rotation;
  cj.origin = Translation3d(p.x, p.y, p.z) *
              Quaterniond(r.w, r.x, r.y, r.z);

  if (cj.is_revolute) {
    cj.axis  = Vector3d(j->axis.x, j->axis.y, j->axis.z).normalized();
    cj.q_idx = q_count++;
    cj.q_min = j->limits ? j->limits->lower : -M_PI;
    cj.q_max = j->limits ? j->limits->upper :  M_PI;
  } else {
    cj.axis  = Vector3d::Zero();
    cj.q_idx = -1;
    cj.q_min = cj.q_max = 0.0;
  }

  chain.push_back(cj);
  return true;
}

// FK: returns foot-tip position in pelvis frame
static Vector3d fk(const std::vector<ChainJoint> &chain, const VectorXd &q) {
  Isometry3d T = Isometry3d::Identity();
  for (const auto &cj : chain) {
    T = T * cj.origin;
    if (cj.is_revolute)
      T = T * AngleAxisd(q[cj.q_idx], cj.axis);
  }
  return T.translation();
}

// Damped-pseudoinverse NR IK with null-space regularization.
//   Task:      minimize || FK(q) - target ||
//   Null-space: simultaneously pull joints toward q_ref (crouch)
//               so the knee stays bent and ankle stays natural.
//
//   dq = J⁺ e  +  (I - J⁺J) · ns_gain · (q_ref - q)
//
// q_ref: desired joint configuration in the null space (e.g. crouch).
//        Pass empty VectorXd to disable null-space term.
static VectorXd nrIK(const std::vector<ChainJoint> &chain,
                     const VectorXd &q0,
                     const Vector3d &target,
                     const VectorXd &q_ref  = VectorXd(),
                     double ns_gain = 0.3,
                     int max_iter   = 80,
                     double tol     = 1e-4,
                     double step    = 0.5,
                     double damp    = 1e-3) {
  VectorXd q = q0;
  const int nq = (int)q.size();
  const double dq_fd = 1e-5;
  const bool use_ns  = (q_ref.size() == nq);

  for (int it = 0; it < max_iter; it++) {
    Vector3d pos = fk(chain, q);
    Vector3d err = target - pos;
    if (err.norm() < tol) break;

    // Numerical Jacobian  3×nq
    MatrixXd J(3, nq);
    for (int i = 0; i < nq; i++) {
      VectorXd qp = q; qp[i] += dq_fd;
      J.col(i) = (fk(chain, qp) - pos) / dq_fd;
    }

    // Damped pseudoinverse: J⁺ = J^T (JJ^T + λI)^{-1}
    Matrix3d JJt = J * J.transpose() + damp * Matrix3d::Identity();
    MatrixXd Jpinv = J.transpose() * JJt.ldlt().solve(Matrix3d::Identity()); // nq×3
    VectorXd dq    = Jpinv * err;

    // Null-space term: (I - J⁺J) · ns_gain · (q_ref - q)
    if (use_ns) {
      MatrixXd N = MatrixXd::Identity(nq, nq) - Jpinv * J;
      dq += N * (ns_gain * (q_ref - q));
    }

    q += step * dq;

    // Clamp to joint limits
    for (const auto &cj : chain) {
      if (cj.is_revolute)
        q[cj.q_idx] = std::max(cj.q_min, std::min(cj.q_max, q[cj.q_idx]));
    }
  }
  return q;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main node
// ─────────────────────────────────────────────────────────────────────────────
class NRIKTestNode : public rclcpp::Node {
public:
  NRIKTestNode() : Node("nr_ik_test") {
    // Crouch joint angles — tune to balance CoM over feet.
    // FK at default: foot_r pelvis ≈ (-0.149, -0.122, -0.702)
    //   hip_pitch  = abduction/adduction
    //   hip_roll   = sagittal (+ pushes foot forward, - pulls back)
    //   knee       = bend (more negative = deeper crouch)
    //   ankle      = compensate (more negative = toes up / lean back)
    c_hip_pitch_           = declare_parameter("crouch_hip_pitch",        -0.35);
    c_hip_roll_            = declare_parameter("crouch_hip_roll",         0.00);
    c_knee_pitch_          = declare_parameter("crouch_knee_pitch",       -0.35);
    c_ankle_pitch_         = declare_parameter("crouch_ankle_pitch",      -0.15);
    settle_shoulder_pitch_  = declare_parameter("settle_shoulder_pitch",   0.0);
    settle_shoulder_roll_r_ = declare_parameter("settle_shoulder_roll_r",  0.3);
    settle_shoulder_roll_l_ = declare_parameter("settle_shoulder_roll_l", -0.3);

    step_length_ = declare_parameter("step_length",  0.06);
    step_height_ = declare_parameter("step_height",  0.04);
    step_dur_    = declare_parameter("step_duration", 5.0);
    settle_dur_  = declare_parameter("settle_duration", 6.0);
    n_steps_     = declare_parameter("n_steps", 6);
    control_hz_  = declare_parameter("control_hz", 20.0);

    // Manual IK target (overrides stepping when all three are set)
    target_x_ = declare_parameter("target_x", 999.0);
    target_y_ = declare_parameter("target_y", 999.0);
    target_z_ = declare_parameter("target_z", 999.0);
    target_leg_ = declare_parameter("target_leg", std::string("right"));

    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr m){ urdfCb(m); });

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/lower_body_controller/joint_trajectory", 10);
    arm_pub_r_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/right_arm_controller/joint_trajectory", 10);
    arm_pub_l_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/left_arm_controller/joint_trajectory", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/nr_ik_test/markers", 10);

    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, this);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_),
      [this]{ controlCb(); });

    startup_ = now();
    RCLCPP_INFO(get_logger(), "nr_ik_test: waiting for robot_description...");
  }

private:
  // ── URDF callback ───────────────────────────────────────────────────────────
  void urdfCb(const std_msgs::msg::String::SharedPtr msg) {
    if (urdf_ready_) return;
    urdf::Model model;
    if (!model.initString(msg->data)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF"); return;
    }

    int qr = 0, ql = 0;
    bool ok_r = buildChain(model, "pelvis", "foot_r", chain_r_, qr);
    bool ok_l = buildChain(model, "pelvis", "foot_l", chain_l_, ql);

    if (!ok_r || !ok_l) {
      RCLCPP_ERROR(get_logger(), "Chain build failed"); return;
    }

    // Build joint-name → index maps for publishing
    for (const auto &cj : chain_r_)
      if (cj.is_revolute) jmap_r_[cj.name] = cj.q_idx;
    for (const auto &cj : chain_l_)
      if (cj.is_revolute) jmap_l_[cj.name] = cj.q_idx;

    nq_r_ = qr; nq_l_ = ql;

    // Starting q: crouch angles (same as backup.cpp)
    // Joint order (by q_idx from chain): hip_pitch_r, hip_roll_r, thigh_yaw_r,
    //   knee_pitch_r, ankle_pitch_r, ankle_roll_r
    q_r_ = VectorXd::Zero(nq_r_);
    q_l_ = VectorXd::Zero(nq_l_);
    applyCrouch(chain_r_, q_r_);
    applyCrouch(chain_l_, q_l_);
    q_r_crouch_ = q_r_;   // save reference for null-space regularization
    q_l_crouch_ = q_l_;

    urdf_ready_ = true;
    RCLCPP_INFO(get_logger(), "Chains built: right=%d joints, left=%d joints", nq_r_, nq_l_);

    // Log FK at crouch so we know where feet are in pelvis frame
    Vector3d pr = fk(chain_r_, q_r_);
    Vector3d pl = fk(chain_l_, q_l_);
    RCLCPP_INFO(get_logger(),
      "Crouch FK: foot_r pelvis=(%.3f %.3f %.3f)  foot_l pelvis=(%.3f %.3f %.3f)",
      pr.x(),pr.y(),pr.z(), pl.x(),pl.y(),pl.z());
  }

  // Apply crouch preset to a joint vector using the chain's q_idx mapping
  void applyCrouch(const std::vector<ChainJoint> &chain, VectorXd &q) {
    const std::map<std::string, double> crouch = {
      {"hip_pitch_r", c_hip_pitch_}, {"hip_roll_r", c_hip_roll_}, {"thigh_yaw_r",  0.0},
      {"knee_pitch_r",   c_knee_pitch_},   {"ankle_pitch_r", c_ankle_pitch_}, {"ankle_roll_r", 0.0},
      {"hip_pitch_l", c_hip_pitch_}, {"hip_roll_l", c_hip_roll_}, {"thigh_yaw_l",  0.0},
      {"knee_pitch_l",   c_knee_pitch_},   {"ankle_pitch_l", c_ankle_pitch_}, {"ankle_roll_l", 0.0},
    };
    for (const auto &cj : chain) {
      if (!cj.is_revolute) continue;
      auto it = crouch.find(cj.name);
      if (it != crouch.end()) q[cj.q_idx] = it->second;
    }
  }

  // ── Control callback ─────────────────────────────────────────────────────────
  void controlCb() {
    double elapsed = (now() - startup_).seconds();

    // Settle: hold crouch + arm pose
    if (!urdf_ready_ || elapsed < settle_dur_) {
      if (urdf_ready_) {
        publishJoints(q_r_, q_l_, 0.5);
        publishArms(0.5);
      }
      return;
    }

    // ── Manual target mode: ros2 param set /nr_ik_test target_x 0.1 etc. ──
    if (target_x_ < 900.0 && target_y_ < 900.0 && target_z_ < 900.0) {
      Vector3d tgt(target_x_, target_y_, target_z_);
      bool right = (target_leg_ != "left");
      if (right) q_r_ = nrIK(chain_r_, q_r_, tgt, q_r_crouch_);
      else       q_l_ = nrIK(chain_l_, q_l_, tgt, q_l_crouch_);
      publishJoints(q_r_, q_l_, 0.2);
      Vector3d fk_out = right ? fk(chain_r_, q_r_) : fk(chain_l_, q_l_);
      publishMarkers(fk(chain_r_, q_r_), fk(chain_l_, q_l_), &tgt);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[manual %s] target=(%.3f %.3f %.3f)  FK=(%.3f %.3f %.3f)  err=%.4f\n"
        "  q: %.3f %.3f %.3f %.3f %.3f %.3f",
        right?"R":"L", tgt.x(),tgt.y(),tgt.z(),
        fk_out.x(),fk_out.y(),fk_out.z(), (tgt-fk_out).norm(),
        right?q_r_[0]:q_l_[0], right?q_r_[1]:q_l_[1], right?q_r_[2]:q_l_[2],
        right?q_r_[3]:q_l_[3], right?q_r_[4]:q_l_[4], right?q_r_[5]:q_l_[5]);
      return;
    }

    // Initialise walk from TF foot positions at first step
    if (!walk_init_) {
      if (!initWalkFromTF()) return;
    }

    // Walk complete
    if (step_idx_ >= n_steps_) {
      publishJoints(q_r_, q_l_, 0.5);
      return;
    }

    double phase = (now() - step_start_).seconds() / step_dur_;
    phase = std::min(1.0, phase);

    // Swing arc: parabolic z clearance, linear x/y
    bool right_swing = (step_idx_ % 2 == 0);
    Vector3d support_target = right_swing ? pL_init_ : pR_init_;
    Vector3d swing_start    = right_swing ? pR_init_ : pL_init_;

    // Target landing = swing_start shifted forward by step_length (in -y direction = walk fwd)
    Vector3d swing_land = swing_start;
    swing_land.y() -= step_length_;    // pelvis -Y = robot forward

    // Current swing foot target with z arc
    double smooth = phase * phase * (3.0 - 2.0 * phase);  // smoothstep
    Vector3d swing_now = swing_start + smooth * (swing_land - swing_start);
    swing_now.z() += step_height_ * std::sin(M_PI * phase); // parabolic lift

    // Solve IK
    if (right_swing) {
      q_r_ = nrIK(chain_r_, q_r_, swing_now,      q_r_crouch_);
      q_l_ = nrIK(chain_l_, q_l_, support_target, q_l_crouch_);
    } else {
      q_l_ = nrIK(chain_l_, q_l_, swing_now,      q_l_crouch_);
      q_r_ = nrIK(chain_r_, q_r_, support_target, q_r_crouch_);
    }

    publishJoints(q_r_, q_l_, 1.0 / control_hz_);

    Vector3d pfk_r = fk(chain_r_, q_r_);
    Vector3d pfk_l = fk(chain_l_, q_l_);
    publishMarkers(pfk_r, pfk_l, nullptr, &swing_now);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "[step %d/%d %s ph=%.2f]\n"
      "  swing_now  (%.3f %.3f %.3f)  land (%.3f %.3f %.3f)\n"
      "  FK right   (%.3f %.3f %.3f)  FK left  (%.3f %.3f %.3f)\n"
      "  q_r: %.2f %.2f %.2f %.2f %.2f %.2f\n"
      "  q_l: %.2f %.2f %.2f %.2f %.2f %.2f",
      step_idx_+1, n_steps_, right_swing ? "R-swing" : "L-swing", phase,
      swing_now.x(),swing_now.y(),swing_now.z(), swing_land.x(),swing_land.y(),swing_land.z(),
      pfk_r.x(),pfk_r.y(),pfk_r.z(), pfk_l.x(),pfk_l.y(),pfk_l.z(),
      q_r_[0],q_r_[1],q_r_[2],q_r_[3],q_r_[4],q_r_[5],
      q_l_[0],q_l_[1],q_l_[2],q_l_[3],q_l_[4],q_l_[5]);

    // Advance step
    if (phase >= 1.0) {
      // Update foot positions after landing and reset IK warm-start to crouch
      // so ankle/knee don't accumulate drift across steps.
      if (right_swing) {
        pR_init_ = swing_land;
      } else {
        pL_init_ = swing_land;
      }
      step_idx_++;
      step_start_ = now();
      RCLCPP_INFO(get_logger(), "Step %d complete. R=(%.3f %.3f)  L=(%.3f %.3f)",
        step_idx_, pR_init_.x(),pR_init_.y(), pL_init_.x(),pL_init_.y());
    }
  }

  // ── Initialise foot positions from TF at walk start ──────────────────────────
  bool initWalkFromTF() {
    try {
      auto tfR = tf_buf_->lookupTransform("pelvis","foot_r",tf2::TimePointZero);
      auto tfL = tf_buf_->lookupTransform("pelvis","foot_l",tf2::TimePointZero);
      pR_init_ = Vector3d(tfR.transform.translation.x,
                          tfR.transform.translation.y,
                          tfR.transform.translation.z);
      pL_init_ = Vector3d(tfL.transform.translation.x,
                          tfL.transform.translation.y,
                          tfL.transform.translation.z);
    } catch (...) { return false; }

    walk_init_  = true;
    step_idx_   = 0;
    step_start_ = now();
    RCLCPP_INFO(get_logger(),
      "Walk start: foot_r pelvis=(%.3f %.3f %.3f)  foot_l=(%.3f %.3f %.3f)",
      pR_init_.x(),pR_init_.y(),pR_init_.z(), pL_init_.x(),pL_init_.y(),pL_init_.z());
    return true;
  }

  // ── Publish RViz2 markers in pelvis frame ────────────────────────────────────
  //   0  : right foot FK  — orange sphere
  //   1  : left  foot FK  — cyan sphere
  //   2  : manual target  — red sphere (only when target is set)
  //   3  : swing target   — yellow sphere (during stepping)
  //   10+: step landing targets — small white cubes
  void publishMarkers(const Vector3d &pfk_r, const Vector3d &pfk_l,
                      const Vector3d *manual_tgt = nullptr,
                      const Vector3d *swing_tgt  = nullptr) {
    visualization_msgs::msg::MarkerArray ma;
    auto t = now();

    auto sphere = [&](int id, const Vector3d &p, float r, float g, float b, float sz = 0.05f) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "pelvis";
      m.header.stamp = t;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x=p.x(); m.pose.position.y=p.y(); m.pose.position.z=p.z();
      m.pose.orientation.w = 1.0;
      m.scale.x=m.scale.y=m.scale.z=sz;
      m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=0.9f;
      return m;
    };

    ma.markers.push_back(sphere(0, pfk_r, 1.0f,0.5f,0.0f));        // orange = R foot
    ma.markers.push_back(sphere(1, pfk_l, 0.0f,0.9f,0.9f));        // cyan   = L foot
    if (manual_tgt)
      ma.markers.push_back(sphere(2, *manual_tgt, 1.0f,0.0f,0.0f, 0.06f)); // red = target
    if (swing_tgt)
      ma.markers.push_back(sphere(3, *swing_tgt,  1.0f,1.0f,0.0f, 0.04f)); // yellow = swing

    // Step landing cubes
    if (walk_init_) {
      auto cube = [&](int id, const Vector3d &p, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "pelvis";
        m.header.stamp = t;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x=p.x(); m.pose.position.y=p.y(); m.pose.position.z=p.z();
        m.pose.orientation.w=1.0;
        m.scale.x=0.08; m.scale.y=0.04; m.scale.z=0.01;
        m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=0.6f;
        return m;
      };
      // Show where feet will land
      Vector3d rp = pR_init_, lp = pL_init_;
      for (int k = 0; k < n_steps_; k++) {
        if (k % 2 == 0) rp.y() -= step_length_;
        else            lp.y() -= step_length_;
        float fr = (k%2==0)?1.0f:0.0f, fg=(k%2==0)?0.5f:0.9f, fb=(k%2==0)?0.0f:0.9f;
        ma.markers.push_back(cube(10+k, k%2==0 ? rp : lp, fr, fg, fb));
      }
    }

    marker_pub_->publish(ma);
  }

  // ── Publish joint trajectory ──────────────────────────────────────────────────
  void publishJoints(const VectorXd &qr, const VectorXd &ql, double dt) {
    // Fixed order matching lower_body_controller
    static const std::vector<std::string> names = {
      "hip_pitch_r","hip_roll_r","thigh_yaw_r","knee_pitch_r","ankle_pitch_r","ankle_roll_r",
      "hip_pitch_l","hip_roll_l","thigh_yaw_l","knee_pitch_l","ankle_pitch_l","ankle_roll_l"
    };

    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    msg.joint_names  = names;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.time_from_start = rclcpp::Duration::from_seconds(dt);
    pt.positions.resize(12, 0.0);

    auto fill = [&](const VectorXd &q,
                    const std::map<std::string,int> &jmap,
                    int offset) {
      for (int i = offset; i < offset+6; i++) {
        auto it = jmap.find(names[i]);
        if (it != jmap.end()) pt.positions[i] = q[it->second];
      }
    };
    fill(qr, jmap_r_, 0);
    fill(ql, jmap_l_, 6);
    msg.points.push_back(pt);
    pub_->publish(msg);
  }

  // ── Publish arm settle pose ───────────────────────────────────────────────────
  void publishArms(double dt) {
    auto make_traj = [&](const std::vector<std::string>& names,
                         const std::vector<double>& positions,
                         rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub) {
      trajectory_msgs::msg::JointTrajectory msg;
      msg.header.stamp = rclcpp::Time(0);
      msg.joint_names  = names;
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.time_from_start = rclcpp::Duration::from_seconds(dt);
      pt.positions = positions;
      msg.points.push_back(pt);
      pub->publish(msg);
    };

    // shoulder_pitch, shoulder_roll, elbow_yaw, elbow_pitch
    make_traj({"shoulder_pitch_r","shoulder_roll_r","elbow_yaw_r","elbow_pitch_r"},
              {settle_shoulder_pitch_, settle_shoulder_roll_r_, 0.0, 0.0}, arm_pub_r_);
    make_traj({"shoulder_pitch_l","shoulder_roll_l","elbow_yaw_l","elbow_pitch_l"},
              {settle_shoulder_pitch_, settle_shoulder_roll_l_, 0.0, 0.0}, arm_pub_l_);
  }

  // ── Members ───────────────────────────────────────────────────────────────────
  std::vector<ChainJoint> chain_r_, chain_l_;
  std::map<std::string,int> jmap_r_, jmap_l_;
  int nq_r_{0}, nq_l_{0};
  VectorXd q_r_, q_l_;
  VectorXd q_r_crouch_, q_l_crouch_;  // null-space reference (crouch pose)

  Vector3d pR_init_, pL_init_;

  bool urdf_ready_{false};
  bool walk_init_{false};
  int  step_idx_{0};
  rclcpp::Time startup_, step_start_;

  double c_hip_pitch_, c_hip_roll_, c_knee_pitch_, c_ankle_pitch_;
  double settle_shoulder_pitch_, settle_shoulder_roll_r_, settle_shoulder_roll_l_;
  double step_length_, step_height_, step_dur_, settle_dur_, control_hz_;
  int    n_steps_;
  double target_x_, target_y_, target_z_;
  std::string target_leg_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_r_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_l_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NRIKTestNode>());
  rclcpp::shutdown();
}
