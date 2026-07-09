#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <urdf/model.h>
#include <vector>

#include <Eigen/Dense>

struct TrajPoint {
  double t, x, xdot, y, ydot;
};

class ComVerifier : public rclcpp::Node {
public:
  ComVerifier()
      : Node("com_verifier"), qos(rclcpp::QoS(1).transient_local()),
        startup_time_(this->now()) ,total_mass_(0.00){

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

    phase_start_time_ = this->now();
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lipm_com_marker", 10);
    walk_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ComVerifier::walkCallback, this));
  }

private:
  Eigen::Vector3d footPositionFromURDF(const urdf::Model& model,
                                        const std::string& foot_name,
                                        const std::string& root_name) {
    std::vector<urdf::JointConstSharedPtr> chain;
    auto link = model.getLink(foot_name);
    while (link && link->name != root_name) {
      if (!link->parent_joint) break;
      chain.push_back(link->parent_joint);
      link = model.getLink(link->parent_joint->parent_link_name);
    }
    std::reverse(chain.begin(), chain.end());

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    for (auto& joint : chain) {
      auto& o = joint->parent_to_joint_origin_transform;
      double qx, qy, qz, qw;
      o.rotation.getQuaternion(qx, qy, qz, qw);
      Eigen::Isometry3d Tj = Eigen::Isometry3d::Identity();
      Tj.translation() = Eigen::Vector3d(o.position.x, o.position.y, o.position.z);
      Tj.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      T = T * Tj;
    }
    return T.translation();
  }

  void urdfCallback(const std_msgs::msg::String::SharedPtr msg) {

    urdf::Model model;

    model.initString(msg->data);
    RCLCPP_INFO(this->get_logger(), "Name %s ", model.getName().c_str());

    for (auto &[name, links] : model.links_)
    {
      if(links->inertial) {
        total_mass_ += links->inertial->mass;
        link_masses_[name] = links->inertial->mass;
        link_com_offsets_[name] = Eigen::Vector3d(
            links->inertial->origin.position.x,
            links->inertial->origin.position.y,
            links->inertial->origin.position.z);
      }
    }
    auto foot_r_pos = footPositionFromURDF(model, "foot_r", "pelvis");
    auto foot_l_pos = footPositionFromURDF(model, "foot_l", "pelvis");
    foot_r_y_ = foot_r_pos.y();
    foot_l_y_ = foot_l_pos.y();
    RCLCPP_INFO(this->get_logger(), "URDF foot y: foot_r=%.4f foot_l=%.4f", foot_r_y_, foot_l_y_);
  }

  struct LegAngles {
    double hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll;
  };

  // Foot position in pelvis frame → joint angles (geometric analytical IK)
  LegAngles legIK(double fx, double fy, double fz, bool is_right) {
    const double hx = -0.149;
    const double hy = is_right ? -0.122 : 0.122;
    const double hz = -0.103;

    const double afo_x = 0.017;
    const double afo_y = is_right ? -0.023 : 0.023;
    const double afo_z = -0.030;

    const double ax = fx - afo_x;
    const double ay = fy - afo_y;
    const double az = fz - afo_z;

    const double dx = ax - hx;
    const double dy = ay - hy;
    const double dz = az - hz;

    const double L1 = 0.2517, L2 = 0.3472;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double cos_knee = (dist * dist - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    cos_knee = std::max(-1.0, std::min(1.0, cos_knee));
    const double knee_pitch = std::acos(cos_knee);

    const double hip_roll  = std::atan2(dy, -dz);
    const double hip_pitch = std::atan2(dx, std::sqrt(dy * dy + dz * dz));

    const double ankle_pitch = -(hip_pitch + knee_pitch);
    const double ankle_roll  = -hip_roll;

    const double ankle_roll_cmd = is_right ? ankle_roll : -ankle_roll;

    constexpr double off_hip_roll   = 0.615;
    constexpr double off_hip_pitch  = 0.063;
    constexpr double off_knee_pitch = 0.389;
    constexpr double off_ankle_pitch = -0.753;

    return {0.0,
            hip_roll - off_hip_roll,
            -(hip_pitch - off_hip_pitch),
            -(knee_pitch - off_knee_pitch),
            (ankle_pitch - off_ankle_pitch),
            ankle_roll_cmd};
  }

  trajectory_msgs::msg::JointTrajectory makeSettleMsg() {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    msg.joint_names = {"hip_pitch_r",  "hip_roll_r",    "thigh_yaw_r",
                       "knee_pitch_r", "ankle_pitch_r", "ankle_roll_r",
                       "hip_pitch_l",  "hip_roll_l",    "thigh_yaw_l",
                       "knee_pitch_l", "ankle_pitch_l", "ankle_roll_l"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    // Analytically derived so the foot sits at pelvis_x = step_x0_ +
    // com_x_pelvis_ = -0.05 + (-0.018) = -0.068, matching the LIPM orbital-IC
    // position. With old {0.0, -0.616, -0.315} the foot was at -0.099 (9.9 cm
    // forward), causing a 3 cm stance-leg mismatch at walk start that drove a
    // forward lurch.
    pt.positions = {-0.615, 0.0, 0.0, -0.615, -0.351, 0.0,
                    -0.615, 0.0, 0.0, -0.615, -0.351, 0.0};
    pt.time_from_start = rclcpp::Duration::from_seconds(1.2);
    msg.points.push_back(pt);
    return msg;
  }

  void walkCallback()
  {
    // Settle phase: send fixed stance until robot settles
    if ((this->now() - startup_time_).seconds() < 3.0) {
      walk_pub_->publish(makeSettleMsg());
      return;
    }

    if (!trajectory_computed_ || total_mass_ == 0.0)
      return;

    // Current desired CoM from LIPM trajectory
    auto elapsed = (this->now() - walk_start_time_).seconds();
    int idx = std::min((int)(elapsed / dt), (int)com_trajectory_.size() - 1);
    double x_des = com_trajectory_[idx].x;
    double y_des = com_trajectory_[idx].y;

    // Resolve which footstep each foot is currently targeting
    int right_step_idx = (current_step % 2 == 0) ? current_step : current_step - 1;
    int left_step_idx  = (current_step % 2 == 1) ? current_step : std::min(current_step + 1, N - 1);
    right_step_idx = std::max(0, right_step_idx);
    left_step_idx  = std::max(0, std::min(left_step_idx, N - 1));

    // Foot positions in pelvis frame:
    // foot_pelvis = (foot_world - com_world) + com_in_pelvis_frame
    double r_fx = (footsteps_[right_step_idx].x - x_des) + com_x_pelvis_;
    double r_fy = (footsteps_[right_step_idx].y - y_des) + com_y_pelvis_;
    double l_fx = (footsteps_[left_step_idx].x  - x_des) + com_x_pelvis_;
    double l_fy = (footsteps_[left_step_idx].y  - y_des) + com_y_pelvis_;
    double foot_z = -height; // foot is 'height' below CoM in pelvis frame

    auto ang_r = legIK(r_fx, r_fy, foot_z, true);
    auto ang_l = legIK(l_fx, l_fy, foot_z, false);

    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    msg.joint_names = {"hip_pitch_r",  "hip_roll_r",    "thigh_yaw_r",
                       "knee_pitch_r", "ankle_pitch_r", "ankle_roll_r",
                       "hip_pitch_l",  "hip_roll_l",    "thigh_yaw_l",
                       "knee_pitch_l", "ankle_pitch_l", "ankle_roll_l"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {ang_r.hip_roll,   ang_r.hip_pitch, ang_r.hip_yaw,
                    ang_r.knee_pitch, ang_r.ankle_pitch, ang_r.ankle_roll,
                    ang_l.hip_roll,   ang_l.hip_pitch,   ang_l.hip_yaw,
                    ang_l.knee_pitch, ang_l.ankle_pitch, ang_l.ankle_roll};
    pt.time_from_start = rclcpp::Duration::from_seconds(0.1);
    msg.points.push_back(pt);
    walk_pub_->publish(msg);
  }
  void initFootsteps() {
    auto fr = tf_buf_->lookupTransform("world", "foot_r", tf2::TimePointZero);
    auto fl = tf_buf_->lookupTransform("world", "foot_l", tf2::TimePointZero);
    double fx0 = (fr.transform.translation.x + fl.transform.translation.x) * 0.5;
    double fy0 = (fr.transform.translation.y + fl.transform.translation.y) * 0.5;
    // y-separation comes from hip_roll joint angle, not fixed geometry — use foot_separation
    foot_r_y_ = fy0 - foot_separation;
    foot_l_y_ = fy0 + foot_separation;
    footsteps_.clear();
    for (int i = 0; i < N; i++) {
      double foot_x = (i <= 1) ? fx0 : fx0 + (i - 1) * step_length;
      footsteps_.push_back({foot_x, (i % 2 == 0) ? foot_r_y_ : foot_l_y_});
    }
    footsteps_initialized_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Footsteps init: fx0=%.3f fy_r=%.4f fy_l=%.4f", fx0, foot_r_y_, foot_l_y_);
  }

  void timerCallback() {
    if (total_mass_ == 0.0)
      return;

    try {
      if (!footsteps_initialized_) {
        initFootsteps();
        return;
      }
      // geometry_msgs::msg::TransformStamped pelvis_tf =
      //     tf_buf_->lookupTransform("world", "pelvis", tf2::TimePointZero);
      // RCLCPP_INFO(this->get_logger(), "Pelvis in world: x:%.3f y:%.3f z:%.3f",
      //             pelvis_tf.transform.translation.x,
      //             pelvis_tf.transform.translation.y,
      //             pelvis_tf.transform.translation.z);

      // Eigen::Vector3d com_(0, 0, 0);
      double com_x_ = 0.0, com_y_ = 0.0, com_z_ = 0.0;
      for (auto &[name, mass] : link_masses_)
      {
        geometry_msgs::msg::TransformStamped transform =
            tf_buf_->lookupTransform("world", name, tf2::TimePointZero);

        // Extract rotation matrix from quaternion
        tf2::Quaternion q(transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z,
                          transform.transform.rotation.w);
        tf2::Matrix3x3 rot(q);

        // Link frame origin in world frame
        Eigen::Vector3d t(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);

        
        // Convert rotation matrix to Eigen
        Eigen::Matrix3d R;
        for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++)
            R(i, j) = rot[i][j];

        // Local CoM offset from link's inertial frame
        Eigen::Vector3d com_local = link_com_offsets_[name];

        // Transform: com_world = R * com_local + t
        Eigen::Vector3d com_world = R * com_local + t;
          com_x_ += mass * com_world(0);
          com_y_ += mass * com_world(1);
          com_z_ += mass * com_world(2);
          // RCLCPP_INFO(this->get_logger(), "Link %s: CoM=[%.3f, %.3f, %.3f] mass=%.3f",
          //             name.c_str(), com_world.x(), com_world.y(), com_world.z(), mass);
      } 
      com_x_ /= total_mass_;
      com_y_ /= total_mass_;
      com_z_ /= total_mass_;
   //   RCLCPP_INFO(this->get_logger(), "Com x:%0.3f Com y:%0.3f, Com z: %0.3f \n", com_x_, com_y_, com_z_);
      geometry_msgs::msg::TransformStamped left_sole_pelvis_tf_ = tf_buf_->lookupTransform("world", "foot_l", tf2::TimePointZero);
      geometry_msgs::msg::TransformStamped right_sole_pelvis_tf_ = tf_buf_->lookupTransform("world", "foot_r", tf2::TimePointZero);
      height = com_z_ - (right_flag_ ? right_sole_pelvis_tf_.transform.translation.z : left_sole_pelvis_tf_.transform.translation.z);
      auto pelvis_tf = tf_buf_->lookupTransform("world", "pelvis", tf2::TimePointZero);
      com_x_pelvis_ = com_x_ - pelvis_tf.transform.translation.x;
      com_y_pelvis_ = com_y_ - pelvis_tf.transform.translation.y;
      if (!initialized_) {
        des_com_x_ = com_x_;
        des_com_y_ = com_y_;
        des_com_xdot_ = 0.0;
        des_com_ydot_ = 0.0;
        initialized_ = true;
      }
      
      // alpha = T / T_ds;
      auto elapsed = (this->now() - phase_start_time_).seconds();
      alpha = std::min(elapsed / T_ds, 1.0);
      omega = sqrt(9.81 / height);

      if (!trajectory_computed_) {
        double wT = omega * T_ss;
        // x: monotonic forward, periodic condition derived from step_length
        double xdot0 = step_length * omega * sinh(wT) / (2.0 * (cosh(wT) - 1.0));
        double x_rel = xdot0 * (1.0 - cosh(wT)) / (omega * sinh(wT));
        // y: alternating ±foot_separation around center, +1 denominator (not -1)
        double y0 = (footsteps_[0].y + footsteps_[1].y) * 0.5;
        double ydot0 = foot_separation * omega * sinh(wT) / (cosh(wT) + 1.0);
        computeTrajectory(footsteps_[0].x + x_rel, y0, xdot0, ydot0);
        walk_start_time_ = this->now();
      }

      com_xdot_ = (com_x_ - prev_com_x) / 0.5;
      com_ydot_ =( com_y_  -prev_com_y)/ 0.5;
      prev_com_x = com_x_;
      prev_com_y = com_y_;
     
    
      switch (current_phase_)
      { // double support phase
      case DOUBLE_SUPPORT:
      {
        // zmp_x = left_sole_pelvis_tf_.transform.translation.x + alpha * (right_sole_pelvis_tf_.transform.translation.x - left_sole_pelvis_tf_.transform.translation.x);
        // zmp_y = left_sole_pelvis_tf_.transform.translation.y + alpha * (right_sole_pelvis_tf_.transform.translation.y - left_sole_pelvis_tf_.transform.translation.y);
        if (current_step + 1 < N) {
          zmp_x = footsteps_[current_step].x + alpha * (footsteps_[current_step + 1].x - footsteps_[current_step].x);
          zmp_y = footsteps_[current_step].y + alpha * (footsteps_[current_step + 1].y - footsteps_[current_step].y);
        } else {
          zmp_x = footsteps_[N - 1].x;
          zmp_y = footsteps_[N - 1].y;
        }

        if (elapsed > T_ds && current_step + 1 < N)
        {
          current_step++;
          current_phase_ = right_flag_ ? SINGLE_RIGHT : SINGLE_LEFT;
          right_flag_ = !right_flag_;
          phase_start_time_ = this->now();
        }
        break;
      }
      case SINGLE_LEFT:
      {
        if (current_step < N) {
          zmp_x = footsteps_[current_step].x;
          zmp_y = footsteps_[current_step].y;
        }
        if (elapsed > T_ss)
        {
          current_phase_ = DOUBLE_SUPPORT;
          phase_start_time_ = this->now();
        }
        break;
      }
      case SINGLE_RIGHT:
      {
        if (current_step < N) {
          zmp_x = footsteps_[current_step].x;
          zmp_y = footsteps_[current_step].y;
        }
        if (elapsed > T_ss)
        {
          current_phase_ = DOUBLE_SUPPORT;
          phase_start_time_ = this->now();
        }
        break;
      }
        }
        x_next = (com_x_ - zmp_x) * cosh(omega * 0.5) + (com_xdot_ / omega) * sinh(omega * 0.5) + zmp_x;
        xdot_next = (com_x_ - zmp_x) * omega * sinh(omega * 0.5) + com_xdot_ * cosh(omega * 0.5);
        y_next = (com_y_ - zmp_y) * cosh(omega * 0.5) + (com_ydot_ / omega) * sinh(omega * 0.5) + zmp_y;
        ydot_next = (com_y_ - zmp_y) * omega * sinh(omega * 0.5) + com_ydot_ * cosh(omega * 0.5);
        RCLCPP_INFO(this->get_logger(),
                    "phase=%d step=%d zmp=(%.3f,%.3f) com=(%.3f,%.3f) next=(%.3f,%.3f)",
                    current_phase_, current_step, zmp_x, zmp_y, com_x_, com_y_, x_next, y_next);
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = y_next;
        marker.pose.position.y = x_next;
        marker.pose.position.z = com_z_;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_pub_->publish(marker);

        if (trajectory_computed_) {
          visualization_msgs::msg::Marker traj_marker;
          traj_marker.header.frame_id = "world";
          traj_marker.header.stamp = this->now();
          traj_marker.id = 1;
          traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          traj_marker.action = visualization_msgs::msg::Marker::ADD;
          traj_marker.pose.orientation.w = 1.0;
          traj_marker.scale.x = 0.01;
          traj_marker.color.r = 0.0;
          traj_marker.color.g = 1.0;
          traj_marker.color.b = 0.0;
          traj_marker.color.a = 1.0;
          for (auto& pt : com_trajectory_) {
            geometry_msgs::msg::Point p;
            p.x = pt.y;
            p.y = -pt.x;
            p.z = com_z_;
            traj_marker.points.push_back(p);
          }
          marker_pub_->publish(traj_marker);
        }
    }
    catch(tf2::TransformException &e) {
      RCLCPP_ERROR(this->get_logger(), "error: %s", e.what());
    }
  }

  

  void computeTrajectory(double x0, double y0, double xdot0, double ydot0) {
    com_trajectory_.clear();
    double x = x0, y = y0, xdot = xdot0, ydot = ydot0;
    com_trajectory_.push_back({0.0, x, xdot, y, ydot});

    for (int i = 0; i + 1 < N; i++) {
      // Single support only: ZMP fixed at footstep[i+1]
      // DS is skipped here — it disrupts the periodic initial conditions
      double zx = footsteps_[i + 1].x;
      double zy = footsteps_[i + 1].y;
      for (double t = 0.0; t < T_ss; t += dt) {
        double x_new    = (x - zx) * cosh(omega * dt) + (xdot / omega) * sinh(omega * dt) + zx;
        double xdot_new = (x - zx) * omega * sinh(omega * dt) + xdot * cosh(omega * dt);
        double y_new    = (y - zy) * cosh(omega * dt) + (ydot / omega) * sinh(omega * dt) + zy;
        double ydot_new = (y - zy) * omega * sinh(omega * dt) + ydot * cosh(omega * dt);
        x = x_new; xdot = xdot_new;
        y = y_new; ydot = ydot_new;
        com_trajectory_.push_back({t, x, xdot, y, ydot});
      }
    }
    trajectory_computed_ = true;
    RCLCPP_INFO(this->get_logger(), "Trajectory computed: %zu points", com_trajectory_.size());
    RCLCPP_INFO(this->get_logger(), "  start: x=%.3f y=%.3f xdot=%.3f ydot=%.3f omega=%.3f",
                x0, y0, xdot0, ydot0, omega);
    RCLCPP_INFO(this->get_logger(), "  end:   x=%.3f y=%.3f xdot=%.3f ydot=%.3f",
                com_trajectory_.back().x, com_trajectory_.back().y,
                com_trajectory_.back().xdot, com_trajectory_.back().ydot);
    for (int i = 0; i < N; i++) {
      RCLCPP_INFO(this->get_logger(), "  footstep[%d]: x=%.3f y=%.3f", i, footsteps_[i].x, footsteps_[i].y);
    }
  }
  std::vector<TrajPoint> com_trajectory_;
  double dt = 0.02;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr< tf2_ros::Buffer>tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
  rclcpp::QoS qos;
  rclcpp::Time startup_time_;
  std::map<std::string, double> link_masses_;
  std::map<std::string, Eigen::Vector3d> link_com_offsets_;
  double total_mass_;
  double height = 0.0;
  double x_ddot = 0.0;
  double T = 0.5;
  double omega = 0.0;
  double T_step = 0.0; // step duration
  double T_ds = 0.6; // double support duration
  double T_ss = 0.8; // single support duration
  double zmp_x = 0.0;
  double zmp_y = 0.0;
  double foot_step = 0.05;
  double foot_separation = 0.05;
  rclcpp::TimerBase::SharedPtr walk_timer_;
  enum phase_
  {
    DOUBLE_SUPPORT,
    SINGLE_LEFT,
    SINGLE_RIGHT
  };
  struct Footstep{
    double x, y;
  };
  std::vector<Footstep> footsteps_;
  
  phase_ current_phase_ = DOUBLE_SUPPORT;
  rclcpp::Time phase_start_time_;
  rclcpp::Time walk_start_time_;
  double alpha = 0.0;
  bool right_flag_ = true;
  double com_xdot_ = 0.0;
  double com_ydot_ = 0.0;
  double prev_com_x = 0.0;
  double prev_com_y = 0.0;
  double x_next = 0.0;
  double xdot_next = 0.0;
  double y_next = 0.0;
  double ydot_next = 0.0;
  double des_com_x_ = 0.0;
  double des_com_y_ = 0.0;
  double des_com_xdot_ = 0.0;
  double des_com_ydot_ = 0.0;
  double com_x_pelvis_ = 0.0;  // CoM x offset in pelvis frame (world_com_x - world_pelvis_x)
  double com_y_pelvis_ = 0.0;  // CoM y offset in pelvis frame
  bool initialized_ = false;
  bool footsteps_initialized_ = false;
  bool trajectory_computed_ = false;
  double foot_r_y_ = 0.0;
  double foot_l_y_ = 0.0;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  double step_length = 0.1;
  int N = 4;
  double zx = 0.0, zy = 0.0;
  int current_step = 0;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr walk_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComVerifier>());
  rclcpp::shutdown();
  return 0;
}
