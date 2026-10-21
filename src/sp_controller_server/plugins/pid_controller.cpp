#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

#include <limits>
#include <cmath>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

}

namespace pid_controller {

void PidController::configure(
    const rclcpp::Node::SharedPtr & node, const std::string & name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_        = node;
  plugin_name_ = name;
  tf_buffer_   = tf_buffer;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }
  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");
  // TODO: 在此读取参数
  lookahead_distance_ = node_->declare_parameter<double>(plugin_name_ + ".lookahead_distance", 0.4);
  if (!std::isfinite(lookahead_distance_) || lookahead_distance_ <= 0.0) {
    throw std::invalid_argument("lookahead_distance must be finite and positive");
  }
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received an empty path");
    return;
  }
  RCLCPP_DEBUG(node_->get_logger(), "Received a path with %zu poses", global_plan_.poses.size());
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &)
{
  // TODO：在此实现路径跟踪
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp    = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  if (global_plan_.poses.empty()) {
    return cmd_vel;  // 返回空速度
  }

  // 遍历寻找路径的最近点
  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;

  std::size_t nearest_index        = 0;
  double      min_distance_squared = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto & point            = global_plan_.poses[i].pose.position;
    const double dx               = point.x - robot_x;
    const double dy               = point.y - robot_y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < min_distance_squared) {
      min_distance_squared = distance_squared;
      nearest_index        = i;
    }
  }

  RCLCPP_DEBUG(
      node_->get_logger(), "Nearest index: %zu,squared distance: %.4f", nearest_index,
      min_distance_squared);

  std::size_t target_index         = nearest_index;
  double      accumulated_distance = 0.0;

  while (target_index + 1 < global_plan_.poses.size() &&
         accumulated_distance < lookahead_distance_) {
    const auto & current = global_plan_.poses[target_index].pose.position;
    const auto & next    = global_plan_.poses[target_index + 1].pose.position;

    accumulated_distance += std::hypot(next.x - current.x, next.y - current.y);
    ++target_index;
  }

  const auto & target = global_plan_.poses[target_index].pose.position;

  RCLCPP_DEBUG(
      node_->get_logger(), "Target index: %zu, position: (%.3f, %.3f)", target_index, target.x,
      target.y);

  RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[%s] Controller plugin not implemented — implement computeVelocityCommands()",
      plugin_name_.c_str());
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
