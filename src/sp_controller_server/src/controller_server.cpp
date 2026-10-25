#include "sp_controller_server/controller_server.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(rclcpp::Node * node, const std::string & name)
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

namespace sp_controller_server {

ControllerServer::ControllerServer()
  : Node("controller_server"),
    loader_("sp_controller_server", "sp_controller_server::ControllerPlugin")
{
  local_path_topic_  = require_param<std::string>(this, "local_path_topic");
  odom_topic_        = require_param<std::string>(this, "odom_topic");
  odom_frame_id_     = require_param<std::string>(this, "odom_frame_id");
  map_frame_id_      = require_param<std::string>(this, "map_frame_id");
  cmd_vel_topic_     = require_param<std::string>(this, "cmd_vel_topic");
  plugin_name_       = require_param<std::string>(this, "plugin_name");
  plugin_type_       = require_param<std::string>(this, "plugin_type");
  base_frame_id_     = require_param<std::string>(this, "base_frame_id");
  control_frequency_ = require_param<double>(this, "control_frequency");

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      local_path_topic_, 10,
      std::bind(&ControllerServer::onLocalPath, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ControllerServer::onOdometry, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  set_control_enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "/set_control_enable", std::bind(
                                 &ControllerServer::onSetControlEnable, this, std::placeholders::_1,
                                 std::placeholders::_2));

  if (control_frequency_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "control_frequency must be positive. Resetting to 10 Hz.");
    control_frequency_ = 10.0;
  }

  control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&ControllerServer::controlLoop, this));
}

void ControllerServer::init()
{
  loadPlugin();
}

void ControllerServer::loadPlugin()
{
  try {
    controller_ = loader_.createSharedInstance(plugin_type_);
    rclcpp::Node::SharedPtr node_ptr(this, [](rclcpp::Node *) {});
    controller_->configure(node_ptr, plugin_name_, tf_buffer_);
    RCLCPP_INFO(
        this->get_logger(), "Loaded controller plugin: %s (%s)", plugin_name_.c_str(),
        plugin_type_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load controller plugin: %s", e.what());
    throw;
  }
}

void ControllerServer::onLocalPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  nav_msgs::msg::Path processed_path = *msg;

  const bool has_embedded_velocity = std::any_of(
      processed_path.poses.begin(), processed_path.poses.end(),
      [](const geometry_msgs::msg::PoseStamped & pose) {
        return std::fabs(pose.pose.orientation.x) > 1e-4 ||
               std::fabs(pose.pose.orientation.y) > 1e-4;
      });

  bool path_changed = true;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_changed = latest_path_.header.frame_id != processed_path.header.frame_id ||
      latest_path_.poses.size() != processed_path.poses.size();
    if (!path_changed) {
      for (std::size_t i = 0; i < processed_path.poses.size(); ++i) {
        const auto & a = latest_path_.poses[i].pose.position;
        const auto & b = processed_path.poses[i].pose.position;
        if (std::abs(a.x-b.x) > 1e-6 || std::abs(a.y-b.y) > 1e-6 ||
            std::abs(a.z-b.z) > 1e-6) {
          path_changed = true;
          break;
        }
      }
    }
  }

  if (controller_ && path_changed) {
    controller_->setPlan(processed_path);
  }

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_path_ = std::move(processed_path);
  }

  if (has_embedded_velocity) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "Local path contains embedded velocity profile in pose.orientation.x/y");
  }
}

void ControllerServer::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_state_mutex_);
  latest_odom_     = *msg;
  have_odom_state_ = true;
}

geometry_msgs::msg::PoseStamped ControllerServer::getCurrentPose(const std::string & frame_id)
{
  if (!tf_buffer_) {
    throw tf2::TransformException("TF buffer is not initialized");
  }

  geometry_msgs::msg::TransformStamped transform;
  transform = tf_buffer_->lookupTransform(frame_id, base_frame_id_, tf2::TimePointZero);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp     = transform.header.stamp;
  pose.header.frame_id  = frame_id;
  pose.pose.position.x  = transform.transform.translation.x;
  pose.pose.position.y  = transform.transform.translation.y;
  pose.pose.position.z  = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

void ControllerServer::controlLoop()
{
  nav_msgs::msg::Path path_copy;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_copy = latest_path_;
  }

  if (path_copy.poses.empty()) {
    return;
  }

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "path_copy succeeded.");

  if (!controller_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Controller plugin not yet available.");
    return;
  }

  nav_msgs::msg::Odometry odom_copy;
  bool                    have_odom = false;
  {
    std::lock_guard<std::mutex> lock(odom_state_mutex_);
    have_odom = have_odom_state_;
    odom_copy = latest_odom_;
  }

  const auto         now = this->now();
  const rclcpp::Time odom_time(odom_copy.header.stamp, now.get_clock_type());
  const double       odom_age     = (now - odom_time).seconds();
  constexpr double   max_odom_age = 0.4;

  if (!have_odom || odom_time.nanoseconds() <= 0 || !std::isfinite(odom_age) || odom_age < 0.0 ||
      odom_age > max_odom_age) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Odometry state missing or stale (age %.3f s); publishing zero velocity.", odom_age);
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  try {
    const auto transform = tf_buffer_->lookupTransform(
        path_copy.header.frame_id, odom_copy.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(odom_copy.pose.pose, current_pose.pose, transform);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot transform odometry pose into path frame: %s; publishing zero velocity.", e.what());
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  current_pose.header.frame_id = path_copy.header.frame_id;
  current_pose.header.stamp    = odom_copy.header.stamp;

  // 本仿真在 /Odometry 中直接发布地图系速度，因此不再按 yaw 旋转。
  geometry_msgs::msg::Twist current_twist = odom_copy.twist.twist;

  geometry_msgs::msg::TwistStamped cmd;
  try {
    cmd = controller_->computeVelocityCommands(current_pose, current_twist);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Computed velocity commands: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Controller plugin threw exception: %s",
        e.what());
    return;
  }

  if (!control_enabled_.load()) {
    cmd.twist = geometry_msgs::msg::Twist{};
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Control disabled, publishing zero velocity.");
  }

  if (cmd.header.frame_id.empty()) {
    cmd.header.frame_id = base_frame_id_;
  }
  cmd.header.stamp = this->now();

  cmd_vel_pub_->publish(cmd.twist);
}

void ControllerServer::onSetControlEnable(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr      response)
{
  control_enabled_.store(request->data);
  response->success = true;
  response->message = request->data ? "Control enabled" : "Control disabled";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

}
