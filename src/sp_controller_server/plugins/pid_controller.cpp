#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

#include <limits>
#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

  position_gain_    = node_->declare_parameter<double>(plugin_name_ + ".position_gain", 1.0);
  max_linear_speed_ = node_->declare_parameter<double>(plugin_name_ + ".max_linear_speed", 0.2);

  if (!std::isfinite(position_gain_) || position_gain_ <= 0.0 ||
      !std::isfinite(max_linear_speed_) || max_linear_speed_ <= 0.0) {
    throw std::invalid_argument("Position gain and speed limit must be finite and positive");
  }

  goal_tolerance_ = node_->declare_parameter<double>(plugin_name_ + ".goal_tolerance", 0.18);

  if (!std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0) {
    throw std::invalid_argument("Goal tolerance must be finite and positive");
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
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity)
{
  // TODO：在此实现路径跟踪
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp    = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  if (global_plan_.poses.empty()) {
    return cmd_vel;  // 返回空速度
  }

  // 检查坐标系一致
  if (global_plan_.header.frame_id.empty() ||
      pose.header.frame_id != global_plan_.header.frame_id) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Pose and path must have the same non-empty frame");
    return cmd_vel;
  }

  // 遍历寻找路径的最近点
  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;

  if (!std::isfinite(robot_x) || !std::isfinite(robot_y)) {
    return cmd_vel;
  }

  std::size_t nearest_index        = 0;
  double      min_distance_squared = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto & point = global_plan_.poses[i].pose.position;

    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return cmd_vel;
    }

    const double dx               = point.x - robot_x;
    const double dy               = point.y - robot_y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < min_distance_squared) {
      min_distance_squared = distance_squared;
      nearest_index        = i;
    }
  }

  const auto & goal          = global_plan_.poses.back().pose.position;
  const double goal_distance = std::hypot(goal.x - robot_x, goal.y - robot_y);

  if (goal_distance <= goal_tolerance_) {
    return cmd_vel;
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

  const double error_x         = target.x - robot_x;
  const double error_y         = target.y - robot_y;
  const double target_distance = std::hypot(error_x, error_y);

  RCLCPP_DEBUG(
      node_->get_logger(), "Target error: (%.3f, %.3f), distance: %.3f", error_x, error_y,
      target_distance);

  double desired_vx = position_gain_ * error_x;
  double desired_vy = position_gain_ * error_y;

  const double desired_speed = std::hypot(desired_vx, desired_vy);

  if (desired_speed > max_linear_speed_) {
    const double scale  = max_linear_speed_ / desired_speed;
    desired_vx         *= scale;
    desired_vy         *= scale;
  }

  RCLCPP_DEBUG(
      node_->get_logger(), "Desired path-frame velocity: (%.3f, %.3f)", desired_vx, desired_vy);

  if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y)) {
    return cmd_vel;
  }

  const double velocity_error_x = desired_vx - velocity.linear.x;
  const double velocity_error_y = desired_vy - velocity.linear.y;

  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Velocity: desired=(%.3f, %.3f), measured=(%.3f, %.3f), error=(%.3f, %.3f)", desired_vx,
      desired_vy, velocity.linear.x, velocity.linear.y, velocity_error_x, velocity_error_y);

  // 速度 P 补偿，增益为本轮实验值。
  constexpr double velocity_gain = 0.3;

  double command_vx = desired_vx + velocity_gain * velocity_error_x;
  double command_vy = desired_vy + velocity_gain * velocity_error_y;

  // 经过严谨的推导，我决定把补偿命令上限设为期望速度上限的两倍
  const double command_speed_limit = 2.0 * max_linear_speed_;
  const double command_speed       = std::hypot(command_vx, command_vy);

  if (!std::isfinite(command_speed)) {
    return cmd_vel;
  }

  if (command_speed > command_speed_limit) {
    const double scale  = command_speed_limit / command_speed;
    command_vx         *= scale;
    command_vy         *= scale;
  }

  const auto & orientation = pose.pose.orientation;

  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w)) {
    return cmd_vel;
  }

  tf2::Quaternion rotation;
  tf2::fromMsg(orientation, rotation);

  if (rotation.length2() < 1e-12) {
    return cmd_vel;
  }

  rotation.normalize();  // 归一化避免无效旋转
  const double yaw = tf2::getYaw(rotation);

  // 进行坐标转换
  const double body_vx = std::cos(yaw) * command_vx + std::sin(yaw) * command_vy;
  const double body_vy = -std::sin(yaw) * command_vx + std::cos(yaw) * command_vy;

  RCLCPP_DEBUG(
      node_->get_logger(), "Yaw: %.3f, body velocity: (%.3f, %.3f)", yaw, body_vx, body_vy);

  cmd_vel.twist.linear.x = body_vx;
  cmd_vel.twist.linear.y = body_vy;

  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
