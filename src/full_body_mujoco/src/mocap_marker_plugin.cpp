// mocap_marker_plugin.cpp
// MuJoCo plugin that moves mocap bodies to match /nr_ik_test/markers.
// Markers arrive in the "pelvis" frame — converted to world frame each step
// using data->xpos[pelvis_id] and data->xmat[pelvis_id].

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <memory>

namespace full_body_mujoco {

// World-frame position for one mocap body
struct MarkerPos { double x{0}, y{0}, z{0}; bool valid{false}; };

class MujocoMarkerPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/) override {
    node_ = node;

    // Cache pelvis body id
    pelvis_id_ = mj_name2id(model, mjOBJ_BODY, "pelvis");
    if (pelvis_id_ < 0) {
      RCLCPP_ERROR(node_->get_logger(), "[MujocoMarkerPlugin] pelvis body not found");
      return false;
    }

    // Cache mocap body ids by name
    auto cache = [&](const char* name) {
      int id = mj_name2id(model, mjOBJ_BODY, name);
      if (id < 0)
        RCLCPP_WARN(node_->get_logger(), "[MujocoMarkerPlugin] body '%s' not in model", name);
      return id;
    };
    id_foot_r_  = cache("viz_foot_r");
    id_foot_l_  = cache("viz_foot_l");
    id_target_  = cache("viz_target");
    id_swing_   = cache("viz_swing");
    id_step_r_[0] = cache("viz_step_r0");
    id_step_r_[1] = cache("viz_step_r1");
    id_step_r_[2] = cache("viz_step_r2");
    id_step_l_[0] = cache("viz_step_l0");
    id_step_l_[1] = cache("viz_step_l1");
    id_step_l_[2] = cache("viz_step_l2");

    sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/nr_ik_test/markers", 10,
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) { markerCb(msg); });

    RCLCPP_INFO(node_->get_logger(),
      "[MujocoMarkerPlugin] initialised — pelvis_id=%d", pelvis_id_);
    return true;
  }

  void update(const mjModel* model, mjData* data) override {
    rclcpp::spin_some(node_);

    // Pelvis world position and rotation (row-major 3×3)
    const double* pp = data->xpos  + 3 * pelvis_id_;
    const double* pr = data->xmat  + 9 * pelvis_id_;

    // Helper: transform pelvis-frame (px,py,pz) → world frame, then set mocap
    auto setMocap = [&](int body_id, const MarkerPos& mp) {
      if (body_id < 0 || !mp.valid) return;
      // mocap index = body_id − 1 (mocap bodies are indexed starting from 1)
      // Actually use mj_body2mocap which maps body index to mocap index
      int mid = model->body_mocapid[body_id];
      if (mid < 0) return;
      double* pos = data->mocap_pos + 3 * mid;
      pos[0] = pp[0] + pr[0]*mp.x + pr[1]*mp.y + pr[2]*mp.z;
      pos[1] = pp[1] + pr[3]*mp.x + pr[4]*mp.y + pr[5]*mp.z;
      pos[2] = pp[2] + pr[6]*mp.x + pr[7]*mp.y + pr[8]*mp.z;
    };

    // Snapshot under lock (lock is brief — just copy POD structs)
    MarkerPos fr, fl, tgt, swg;
    std::array<MarkerPos,3> sr, sl;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      fr  = pos_foot_r_;
      fl  = pos_foot_l_;
      tgt = pos_target_;
      swg = pos_swing_;
      sr  = pos_step_r_;
      sl  = pos_step_l_;
    }

    setMocap(id_foot_r_, fr);
    setMocap(id_foot_l_, fl);
    setMocap(id_target_, tgt);
    setMocap(id_swing_,  swg);
    for (int i = 0; i < 3; i++) {
      setMocap(id_step_r_[i], sr[i]);
      setMocap(id_step_l_[i], sl[i]);
    }
  }

  void cleanup() override {
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    sub_.reset();
  }

private:
  void markerCb(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
    // Running on rclcpp executor thread — safe to lock briefly
    std::lock_guard<std::mutex> lk(mtx_);
    int r_step = 0, l_step = 0;
    for (const auto& m : msg->markers) {
      if (m.action == visualization_msgs::msg::Marker::DELETE) continue;
      MarkerPos p;
      p.x = m.pose.position.x;
      p.y = m.pose.position.y;
      p.z = m.pose.position.z;
      p.valid = true;
      switch (m.id) {
        case 0: pos_foot_r_ = p; break;
        case 1: pos_foot_l_ = p; break;
        case 2: pos_target_ = p; break;
        case 3: pos_swing_  = p; break;
        default:
          // IDs 10+ are footstep cubes: even=right, odd=left
          if (m.id >= 10) {
            int idx = m.id - 10;
            if (idx % 2 == 0 && r_step < 3) pos_step_r_[r_step++] = p;
            else if (idx % 2 == 1 && l_step < 3) pos_step_l_[l_step++] = p;
          }
          break;
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;

  int pelvis_id_{-1};
  int id_foot_r_{-1}, id_foot_l_{-1}, id_target_{-1}, id_swing_{-1};
  std::array<int,3> id_step_r_{-1,-1,-1}, id_step_l_{-1,-1,-1};

  std::mutex mtx_;
  MarkerPos pos_foot_r_, pos_foot_l_, pos_target_, pos_swing_;
  std::array<MarkerPos,3> pos_step_r_{}, pos_step_l_{};
};

}  // namespace full_body_mujoco

PLUGINLIB_EXPORT_CLASS(
  full_body_mujoco::MujocoMarkerPlugin,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
