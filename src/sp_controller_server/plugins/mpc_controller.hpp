#pragma once

#include "sp_controller_server/controller_plugin.hpp"
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex>
#include <string>
#include <vector>

namespace mpc_controller {
using Point = Eigen::Vector2d;

struct State {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
};
struct Reference {
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
};
struct SolverWeights {
  double position{12.0}, velocity{2.0}, control{0.05}, slew{0.1}, terminal{4.0};
};
struct SolverConfig {
  double        dt{0.1};
  int           horizon{20};
  double        tau{0.5}, max_speed{0.35}, max_acceleration{0.6};
  SolverWeights weights;
  int           max_iterations{100};
  double        tolerance{1e-4};
};
struct SolverResult {
  std::vector<Eigen::Vector2d> controls;
  std::vector<State>           states;
  bool                         success{false}, converged{false};
  double                       objective{std::numeric_limits<double>::infinity()};
  int                          iterations{0};
};
class MpcSolver {
public:
  explicit MpcSolver(const SolverConfig & config = SolverConfig());
  const SolverConfig & config() const
  {
    return config_;
  }
  bool valid() const
  {
    return valid_;
  }
  SolverResult solve(
      const State & initial, const std::vector<Reference> & references,
      const Eigen::Vector2d & previous_command) const;
  static State step(const State & state, const Eigen::Vector2d & command, double dt, double tau);

private:
  SolverConfig                 config_;
  bool                         valid_{false};
  Eigen::MatrixXd              transition_, control_map_, difference_, hessian_, system_;
  Eigen::VectorXd              state_weights_;
  Eigen::LDLT<Eigen::MatrixXd> factor_;
  double                       rho_{1.0};
};

class SafetyGrid {
public:
  SafetyGrid() = default;
  explicit SafetyGrid(
      const nav_msgs::msg::OccupancyGrid & global,
      const nav_msgs::msg::OccupancyGrid * local = nullptr);
  bool                ready() const;
  const std::string & frame() const;
  bool                pointFree(const Point & point) const;
  bool                segmentFree(const Point & from, const Point & to) const;
  int                 cost(const Point & point) const;
  int                 segmentCost(const Point & from, const Point & to) const;

private:
  struct Layer {
    nav_msgs::msg::OccupancyGrid map;
    double                       cosine{1.0}, sine{0.0};
    bool                         valid{false};
  };
  static Layer makeLayer(const nav_msgs::msg::OccupancyGrid & map);
  static Point toGrid(const Layer & layer, const Point & point);
  static int   cellCost(const Layer & layer, long long x, long long y, bool outside_free);
  static int   gridPointCost(const Layer & layer, const Point & point, bool outside_free);
  static int   layerSegmentCost(
      const Layer & layer, const Point & from, const Point & to, bool outside_free);
  Layer global_, local_;
  bool  has_local_{false};
};

struct TrajectoryOptimizerConfig {
  double spacing{0.04};
  double max_displacement{0.1};
  int    iterations{12};
  double smooth_weight{0.3}, tether_weight{0.15}, cost_weight{0.02};
};
class TrajectoryOptimizer {
public:
  static std::vector<Point> optimize(
      const std::vector<Point> & path, const SafetyGrid & grid,
      const TrajectoryOptimizerConfig & config = TrajectoryOptimizerConfig{});
  static bool collisionFree(const std::vector<Point> & path, const SafetyGrid & grid);
};

class MpcController : public sp_controller_server::ControllerPlugin {
public:
  void configure(
      const rclcpp::Node::SharedPtr & node, const std::string & name,
      const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;
  void                             setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist &       velocity) override;

private:
  struct Projection {
    double arc{0.0}, distance{0.0};
  };
  Projection project(const Point & position) const;
  Reference  sample(double arc) const;
  double     speedAt(double arc) const;
  bool       preparePath(const SafetyGrid & grid);
  void       publishPaths(
      const rclcpp::Time & stamp, const std::string & frame, const std::vector<State> & predicted);
  geometry_msgs::msg::TwistStamped stop(const rclcpp::Time & now, const char * reason);
  geometry_msgs::msg::TwistStamped hold(const rclcpp::Time & now, double command_dt,
                                        const char * reason);

  rclcpp::Node::SharedPtr                    node_;
  std::unique_ptr<MpcSolver>                 solver_;
  SolverConfig                               config_;
  TrajectoryOptimizerConfig                  smoother_;
  std::string                                base_frame_, path_frame_;
  std::mutex                                 mutex_;
  nav_msgs::msg::Path                        raw_path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr    global_map_, local_map_;
  geometry_msgs::msg::PoseStamped::SharedPtr exact_goal_;
  rclcpp::Time                               global_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time                               local_received_{0, 0, RCL_ROS_TIME};
  bool                                       dirty_{true};
  std::vector<Point>                         path_;
  std::vector<double>                        arc_, speeds_;
  double       cruise_speed_{0.30}, lateral_acceleration_{0.25}, braking_acceleration_{0.25};
  double       goal_tolerance_{0.03}, goal_speed_tolerance_{0.025};
  double       actuator_delay_{0.15}, global_timeout_{3.0}, local_timeout_{1.0};
  double       base_uncertainty_{0.025}, max_tracking_error_{0.5};
  Point        previous_command_{Point::Zero()}, previous_measured_velocity_{Point::Zero()};
  double       model_error_{0.0}, yaw_rate_{0.0}, previous_yaw_{0.0};
  std::int64_t measurement_stamp_{0}, command_stamp_{0};
  bool         settled_{false};
  std::int64_t visualization_stamp_{0};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    global_sub_, local_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                smooth_pub_, predicted_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   metrics_pub_;
};
}  // namespace mpc_controller
