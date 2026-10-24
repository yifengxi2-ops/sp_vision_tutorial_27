#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {

class PidController : public sp_controller_server::ControllerPlugin {
public:
  PidController()           = default;
  ~PidController() override = default;

  void configure(
      const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
      const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist &       velocity) override;

private:
  std::string                      plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr          node_;
  nav_msgs::msg::Path              global_plan_;
  std::string                      base_frame_id_;
  double                           lookahead_distance_{0.4};
  double                           position_gain_{1.0};
  double                           max_linear_speed_{0.2};
  double                           goal_tolerance_{0.18};
  // 与路径点一一对应的地图系参考速度。
  std::vector<geometry_msgs::msg::Twist> reference_velocity_profile_;

  double velocity_gain_{0.3};
  // 仅在收到不同时间戳的位姿时更新 yaw 变化率。
  bool         have_previous_yaw_{false};
  std::int64_t previous_pose_stamp_ns_{0};
  double       previous_yaw_{0.0};
  double       estimated_yaw_rate_{0.0};
};

}

#endif
