#include "mpc_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

namespace mpc_controller {
namespace {
constexpr double kPredictionFullSpeedAge = 0.2;
constexpr double kPredictionStopAge = 0.4;
bool positive(double x) { return std::isfinite(x) && x > 0.0; }
bool nonnegative(double x) { return std::isfinite(x) && x >= 0.0; }

Eigen::Vector2d projectBall(const Eigen::Vector2d & x, double radius)
{
  const double norm = x.stableNorm();
  return norm > radius ? x * (radius / norm) : x;
}

bool finite(const State & state)
{
  return state.position.allFinite() && state.velocity.allFinite();
}
}  // namespace

State MpcSolver::step(const State & state, const Eigen::Vector2d & command,
                      double dt, double tau)
{
  State next;
  const double one_minus_a = -std::expm1(-dt / tau);
  next.position = state.position + tau * one_minus_a * state.velocity +
                  (dt - tau * one_minus_a) * command;
  next.velocity = (1.0 - one_minus_a) * state.velocity + one_minus_a * command;
  return next;
}

MpcSolver::MpcSolver(const SolverConfig & config) : config_(config)
{
  const auto & w = config_.weights;
  if (!positive(config_.dt) || !positive(config_.tau) ||
      !positive(config_.max_speed) || !positive(config_.max_acceleration) ||
      !positive(config_.tolerance) || config_.horizon < 1 || config_.horizon > 200 ||
      config_.max_iterations < 1 || config_.max_iterations > 1000 ||
      !nonnegative(w.position) || !nonnegative(w.velocity) || !positive(w.control) ||
      !nonnegative(w.slew) || !nonnegative(w.terminal)) {
    return;
  }

  const int n = config_.horizon;
  const double c = -std::expm1(-config_.dt / config_.tau);
  Eigen::Matrix4d a = Eigen::Matrix4d::Identity();
  a.block<2, 2>(0, 2) = config_.tau * c * Eigen::Matrix2d::Identity();
  a.block<2, 2>(2, 2) = (1.0 - c) * Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 4, 2> b;
  b.topRows<2>() = (config_.dt - config_.tau * c) * Eigen::Matrix2d::Identity();
  b.bottomRows<2>() = c * Eigen::Matrix2d::Identity();

  transition_ = Eigen::MatrixXd::Zero(4 * n, 4);
  control_map_ = Eigen::MatrixXd::Zero(4 * n, 2 * n);
  difference_ = Eigen::MatrixXd::Identity(2 * n, 2 * n);
  state_weights_.resize(4 * n);
  Eigen::Matrix4d power = Eigen::Matrix4d::Identity();
  for (int k = 0; k < n; ++k) {
    power = (a * power).eval();
    transition_.block<4, 4>(4 * k, 0) = power;
    if (k > 0) {
      control_map_.middleRows(4 * k, 4) =
        a * control_map_.middleRows(4 * (k - 1), 4);
      difference_.block<2, 2>(2 * k, 2 * (k - 1)) = -Eigen::Matrix2d::Identity();
    }
    control_map_.block<4, 2>(4 * k, 2 * k) = b;
    const double terminal_scale = k == n - 1 ? 1.0 + w.terminal : 1.0;
    state_weights_.segment<2>(4 * k).setConstant(terminal_scale * w.position);
    state_weights_.segment<2>(4 * k + 2).setConstant(terminal_scale * w.velocity);
  }

  const Eigen::MatrixXd dtd = difference_.transpose() * difference_;
  hessian_ = 2.0 * (control_map_.transpose() * state_weights_.asDiagonal() * control_map_ +
                     w.slew * dtd);
  hessian_.diagonal().array() += 2.0 * w.control;
  rho_ = std::max(1.0, hessian_.diagonal().mean());
  system_ = hessian_ + rho_ * dtd;
  system_.diagonal().array() += rho_;
  if (!transition_.allFinite() || !control_map_.allFinite() ||
      !state_weights_.allFinite() || !system_.allFinite()) {
    return;
  }
  factor_.compute(system_);
  valid_ = factor_.info() == Eigen::Success && factor_.isPositive();
}

SolverResult MpcSolver::solve(const State & initial,
                              const std::vector<Reference> & references,
                              const Eigen::Vector2d & previous_command) const
{
  SolverResult result;
  const int n = config_.horizon;
  if (!valid_ || !finite(initial) || !previous_command.allFinite() ||
      previous_command.stableNorm() > config_.max_speed + 1e-9 ||
      references.size() != static_cast<std::size_t>(n)) {
    return result;
  }

  Eigen::Vector4d x0;
  x0 << initial.position, initial.velocity;
  Eigen::VectorXd target(4 * n);
  for (int k = 0; k < n; ++k) {
    if (!references[k].position.allFinite() || !references[k].velocity.allFinite()) {
      return result;
    }
    target.segment<2>(4 * k) = references[k].position;
    target.segment<2>(4 * k + 2) = references[k].velocity;
  }

  Eigen::VectorXd offset = Eigen::VectorXd::Zero(2 * n);
  offset.head<2>() = previous_command;
  const Eigen::VectorXd free_error = transition_ * x0 - target;
  const Eigen::VectorXd gradient = 2.0 *
    (control_map_.transpose() * state_weights_.asDiagonal() * free_error -
     config_.weights.slew * difference_.transpose() * offset);
  if (!gradient.allFinite()) {
    return result;
  }

  Eigen::VectorXd u(2 * n);
  for (int k = 0; k < n; ++k) {
    u.segment<2>(2 * k) = previous_command;
  }
  const Eigen::VectorXd seed = u;
  Eigen::VectorXd z = u;
  Eigen::VectorXd v = difference_ * u - offset;
  Eigen::VectorXd dual_z = Eigen::VectorXd::Zero(2 * n);
  Eigen::VectorXd dual_v = dual_z;
  const double slew = config_.max_acceleration * config_.dt;

  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    const Eigen::VectorXd rhs = -gradient + rho_ *
      (z - dual_z + difference_.transpose() * (offset + v - dual_v));
    u = factor_.solve(rhs);
    result.iterations = iteration + 1;
    if (factor_.info() != Eigen::Success || !u.allFinite()) {
      return result;
    }
    const Eigen::VectorXd du = difference_ * u - offset;
    const Eigen::VectorXd old_z = z;
    const Eigen::VectorXd old_v = v;
    for (int k = 0; k < n; ++k) {
      z.segment<2>(2 * k) = projectBall(
        u.segment<2>(2 * k) + dual_z.segment<2>(2 * k), config_.max_speed);
      v.segment<2>(2 * k) = projectBall(
        du.segment<2>(2 * k) + dual_v.segment<2>(2 * k), slew);
    }
    dual_z += u - z;
    dual_v += du - v;
    const double primal = std::max((u - z).lpNorm<Eigen::Infinity>(),
                                   (du - v).lpNorm<Eigen::Infinity>());
    const Eigen::VectorXd dual_change = rho_ *
      (z - old_z + difference_.transpose() * (v - old_v));
    if (primal <= config_.tolerance &&
        dual_change.lpNorm<Eigen::Infinity>() <= config_.tolerance) {
      result.converged = true;
      break;
    }
  }

  // Restore exact feasibility after the finite ADMM iteration budget.
  Eigen::Vector2d last = previous_command;
  for (int k = 0; k < n; ++k) {
    const Eigen::Vector2d speed_limited = projectBall(u.segment<2>(2 * k), config_.max_speed);
    u.segment<2>(2 * k) = last + projectBall(speed_limited - last, slew);
    if (!u.segment<2>(2 * k).allFinite() ||
        u.segment<2>(2 * k).stableNorm() > config_.max_speed + 1e-9 ||
        (u.segment<2>(2 * k) - last).stableNorm() > slew + 1e-9) {
      result.converged = false;
      return result;
    }
    last = u.segment<2>(2 * k);
  }

  auto objective = [&](const Eigen::VectorXd & command) {
    const Eigen::VectorXd error = free_error + control_map_ * command;
    return error.dot(state_weights_.asDiagonal() * error) +
           config_.weights.control * command.squaredNorm() +
           config_.weights.slew * (difference_ * command - offset).squaredNorm();
  };
  const double value = objective(u);
  const double baseline = objective(seed);
  if (!std::isfinite(value) || !std::isfinite(baseline) ||
      value > baseline + 1e-8 * (1.0 + baseline)) {
    result.converged = false;
    return result;
  }

  result.states.reserve(n + 1);
  result.states.push_back(initial);
  for (int k = 0; k < n; ++k) {
    result.states.push_back(step(result.states.back(), u.segment<2>(2 * k),
                                config_.dt, config_.tau));
    if (!finite(result.states.back())) {
      result.converged = false;
      result.states.clear();
      return result;
    }
  }
  result.controls.reserve(n);
  for (int k = 0; k < n; ++k) {
    result.controls.emplace_back(u.segment<2>(2 * k));
  }
  result.objective = value;
  result.success = true;
  return result;
}

}  // namespace mpc_controller
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace mpc_controller {
namespace {
constexpr double kBoundaryTolerance = 1e-9;
constexpr std::size_t kMaxOutputPoints = 1000000;
bool finite(const Point & point) { return point.allFinite(); }
}

SafetyGrid::Layer SafetyGrid::makeLayer(const nav_msgs::msg::OccupancyGrid & map)
{
  Layer layer;
  layer.map = map;
  const auto & info = map.info;
  const auto & p = info.origin.position;
  const auto & q = info.origin.orientation;
  const std::uint64_t size = static_cast<std::uint64_t>(info.width) * info.height;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (map.header.frame_id.empty() || info.width == 0 || info.height == 0 ||
      !std::isfinite(info.resolution) || info.resolution <= 0.0 ||
      size != map.data.size() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3 ||
      std::abs(q.x) > 1e-8 || std::abs(q.y) > 1e-8) {
    return layer;
  }
  const double yaw = std::atan2(2.0*q.w*q.z, q.w*q.w - q.z*q.z);
  if (!std::isfinite(yaw)) { return layer; }
  layer.cosine = std::cos(yaw);
  layer.sine = std::sin(yaw);
  layer.valid = true;
  return layer;
}

SafetyGrid::SafetyGrid(const nav_msgs::msg::OccupancyGrid & global,
                       const nav_msgs::msg::OccupancyGrid * local)
: global_(makeLayer(global)), has_local_(local != nullptr)
{
  if (local) { local_ = makeLayer(*local); }
}

bool SafetyGrid::ready() const
{
  return global_.valid && (!has_local_ ||
      (local_.valid && local_.map.header.frame_id == global_.map.header.frame_id));
}

const std::string & SafetyGrid::frame() const { return global_.map.header.frame_id; }

Point SafetyGrid::toGrid(const Layer & layer, const Point & point)
{
  const double dx = point.x() - layer.map.info.origin.position.x;
  const double dy = point.y() - layer.map.info.origin.position.y;
  return Point((layer.cosine*dx + layer.sine*dy) / layer.map.info.resolution,
               (-layer.sine*dx + layer.cosine*dy) / layer.map.info.resolution);
}

int SafetyGrid::cellCost(const Layer & layer, long long x, long long y, bool outside_free)
{
  if (x < 0 || y < 0 || x >= layer.map.info.width || y >= layer.map.info.height) {
    return outside_free ? 0 : 100;
  }
  const auto index = static_cast<std::size_t>(y) * layer.map.info.width +
                     static_cast<std::size_t>(x);
  const int value = layer.map.data[index];
  return value < 0 ? 100 : std::min(value, 100);
}

int SafetyGrid::gridPointCost(const Layer & layer, const Point & point, bool outside_free)
{
  if (!finite(point)) { return 100; }
  if (point.x() < -1.0 || point.y() < -1.0 ||
      point.x() > static_cast<double>(layer.map.info.width) + 1.0 ||
      point.y() > static_cast<double>(layer.map.info.height) + 1.0) {
    return outside_free ? 0 : 100;
  }
  const auto x = static_cast<long long>(std::floor(point.x()));
  const auto y = static_cast<long long>(std::floor(point.y()));
  long long xs[2] = {x, x};
  long long ys[2] = {y, y};
  int nx = 1;
  int ny = 1;
  // Closed-cell supercover: edges touch both cells and vertices all four.
  const double rx = std::round(point.x());
  const double ry = std::round(point.y());
  if (std::abs(point.x() - rx) <= kBoundaryTolerance) {
    xs[0] = static_cast<long long>(rx) - 1;
    xs[1] = static_cast<long long>(rx);
    nx = 2;
  }
  if (std::abs(point.y() - ry) <= kBoundaryTolerance) {
    ys[0] = static_cast<long long>(ry) - 1;
    ys[1] = static_cast<long long>(ry);
    ny = 2;
  }
  int result = 0;
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      result = std::max(result, cellCost(layer, xs[i], ys[j], outside_free));
    }
  }
  return result;
}

int SafetyGrid::layerSegmentCost(const Layer & layer, const Point & from,
                                 const Point & to, bool outside_free)
{
  const Point a = toGrid(layer, from);
  const Point b = toGrid(layer, to);
  const Point delta = b - a;
  if (!finite(a) || !finite(b) || !finite(delta)) { return 100; }
  int result = std::max(gridPointCost(layer, a, outside_free),
                        gridPointCost(layer, b, outside_free));
  if (result >= 100) { return result; }
  // Partition by exact grid crossings, checking both crossings and interiors.
  std::vector<double> cuts{0.0, 1.0};
  for (int axis = 0; axis < 2; ++axis) {
    if (delta[axis] == 0.0) { continue; }
    const double limit = axis == 0 ? layer.map.info.width : layer.map.info.height;
    const double low = std::max(0.0, std::min(a[axis], b[axis]));
    const double high = std::min(limit, std::max(a[axis], b[axis]));
    if (low > high) { continue; }
    const auto begin = static_cast<long long>(std::ceil(low));
    const auto end = static_cast<long long>(std::floor(high));
    for (long long boundary = begin; boundary <= end; ++boundary) {
      const double t = (static_cast<double>(boundary) - a[axis]) / delta[axis];
      if (t > 0.0 && t < 1.0) { cuts.push_back(t); }
    }
  }
  std::sort(cuts.begin(), cuts.end());
  cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
  for (std::size_t i = 0; i < cuts.size(); ++i) {
    result = std::max(result, gridPointCost(layer, a + cuts[i]*delta, outside_free));
    if (i > 0) {
      const double midpoint = cuts[i-1] + (cuts[i] - cuts[i-1])*0.5;
      result = std::max(result, gridPointCost(layer, a + midpoint*delta, outside_free));
    }
    if (result >= 100) { return result; }
  }
  return result;
}

int SafetyGrid::cost(const Point & point) const
{
  if (!ready() || !finite(point)) { return 100; }
  int result = gridPointCost(global_, toGrid(global_, point), false);
  if (has_local_) {
    result = std::max(result, gridPointCost(local_, toGrid(local_, point), true));
  }
  return result;
}

int SafetyGrid::segmentCost(const Point & from, const Point & to) const
{
  if (!ready() || !finite(from) || !finite(to)) { return 100; }
  int result = layerSegmentCost(global_, from, to, false);
  if (result < 100 && has_local_) {
    result = std::max(result, layerSegmentCost(local_, from, to, true));
  }
  return result;
}

bool SafetyGrid::pointFree(const Point & point) const { return cost(point) < 100; }
bool SafetyGrid::segmentFree(const Point & from, const Point & to) const
{
  return segmentCost(from, to) < 100;
}

bool TrajectoryOptimizer::collisionFree(const std::vector<Point> & path, const SafetyGrid & grid)
{
  if (!grid.ready() || path.empty()) { return false; }
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!finite(path[i]) || !grid.pointFree(path[i])) { return false; }
    if (i > 0 && !grid.segmentFree(path[i-1], path[i])) { return false; }
  }
  return true;
}

std::vector<Point> TrajectoryOptimizer::optimize(
    const std::vector<Point> & path, const SafetyGrid & grid,
    const TrajectoryOptimizerConfig & config)
{
  if (!std::isfinite(config.spacing) || config.spacing <= 0.0 ||
      !std::isfinite(config.max_displacement) || config.max_displacement < 0.0 ||
      config.iterations < 0 || config.iterations > 1000 ||
      !std::isfinite(config.smooth_weight) || config.smooth_weight < 0.0 ||
      config.smooth_weight > 1.0 || !std::isfinite(config.tether_weight) ||
      config.tether_weight < 0.0 || config.tether_weight > 1.0 ||
      !std::isfinite(config.cost_weight) || config.cost_weight < 0.0 ||
      path.size() > kMaxOutputPoints || !collisionFree(path, grid)) {
    return {};
  }
  std::vector<Point> original;
  original.reserve(path.size());
  for (const auto & point : path) {
    if (original.empty() || point.x() != original.back().x() ||
        point.y() != original.back().y()) {
      original.push_back(point);
    }
  }
  if (original.size() <= 1) { return original; }
  std::vector<Point> smoothed = original;
  for (int iteration = 0; iteration < config.iterations; ++iteration) {
    bool changed = false;
    for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
      const Point before = smoothed[i];
      const Point midpoint = 0.5*smoothed[i-1] + 0.5*smoothed[i+1];
      Point candidate = before + config.smooth_weight*(midpoint - before) +
                        config.tether_weight*(original[i] - before);
      Point displacement = candidate - original[i];
      const double distance = displacement.norm();
      if (!finite(candidate) || !std::isfinite(distance)) { continue; }
      if (distance > config.max_displacement && distance > 0.0) {
        candidate = original[i] + displacement*(config.max_displacement / distance);
      }
      const int old_cost = grid.cost(before);
      const int new_cost = grid.cost(candidate);
      const int old_left = grid.segmentCost(smoothed[i-1], before);
      const int old_right = grid.segmentCost(before, smoothed[i+1]);
      const int new_left = grid.segmentCost(smoothed[i-1], candidate);
      const int new_right = grid.segmentCost(candidate, smoothed[i+1]);
      if (new_cost > old_cost || new_left >= 100 || new_right >= 100 ||
          new_left > old_left || new_right > old_right) {
        continue;
      }
      const auto energy = [&](const Point & point, int point_cost) {
        return config.smooth_weight*(point - midpoint).squaredNorm() +
               config.tether_weight*(point - original[i]).squaredNorm() +
               config.cost_weight*static_cast<double>(point_cost)/100.0;
      };
      if (energy(candidate, new_cost) + 1e-14 < energy(before, old_cost)) {
        smoothed[i] = candidate;
        changed = true;
      }
    }
    if (!changed) { break; }
  }
  if (!collisionFree(smoothed, grid)) { return {}; }
  std::vector<Point> result;
  result.push_back(smoothed.front());
  // Preserve every breakpoint: resampling must never shortcut a corner.
  for (std::size_t i = 1; i < smoothed.size(); ++i) {
    const Point delta = smoothed[i] - smoothed[i-1];
    const double length = delta.norm();
    const double pieces_double = std::ceil(length / config.spacing);
    if (!std::isfinite(pieces_double) || pieces_double > kMaxOutputPoints) { return {}; }
    const auto pieces = static_cast<std::size_t>(std::max(1.0, pieces_double));
    if (pieces > kMaxOutputPoints - result.size()) { return {}; }
    for (std::size_t part = 1; part <= pieces; ++part) {
      result.push_back(part == pieces ? smoothed[i] :
          Point(smoothed[i-1] + delta*(static_cast<double>(part) / pieces)));
    }
  }
  if (!collisionFree(result, grid)) { return {}; }
  return result;
}
}  // namespace mpc_controller
namespace mpc_controller {
namespace {
Point limit(const Point & value, double maximum)
{
  const double norm = value.norm();
  return norm > maximum ? Point(value * (maximum / norm)) : value;
}
bool valid_positive(double value) { return std::isfinite(value) && value > 0.0; }
}

void MpcController::configure(const rclcpp::Node::SharedPtr & node, const std::string & name,
                             const std::shared_ptr<tf2_ros::Buffer> &)
{
  if (!node) { throw std::invalid_argument("MPC requires a node"); }
  node_ = node;
  auto number = [&](const std::string & key, double fallback) {
    return node_->declare_parameter<double>(name + "." + key, fallback);
  };
  base_frame_ = node_->declare_parameter<std::string>(name + ".base_frame_id", "base_link");
  config_.dt = number("prediction_dt", 0.1);
  config_.horizon = node_->declare_parameter<int>(name + ".horizon", 20);
  config_.tau = number("velocity_time_constant", 0.5);
  config_.max_speed = number("max_linear_speed", 0.35);
  config_.max_acceleration = number("max_command_acceleration", 0.6);
  config_.weights.position = number("position_weight", 12.0);
  config_.weights.velocity = number("velocity_weight", 2.0);
  config_.weights.control = number("control_weight", 0.05);
  config_.weights.slew = number("slew_weight", 0.1);
  config_.weights.terminal = number("terminal_weight", 4.0);
  config_.max_iterations = node_->declare_parameter<int>(name + ".solver_iterations", 80);
  config_.tolerance = number("solver_tolerance", 0.0001);
  cruise_speed_ = number("cruise_speed", 0.30);
  lateral_acceleration_ = number("lateral_acceleration", 0.25);
  braking_acceleration_ = number("braking_acceleration", 0.25);
  goal_tolerance_ = number("goal_tolerance", 0.03);
  goal_speed_tolerance_ = number("goal_speed_tolerance", 0.025);
  actuator_delay_ = number("actuator_delay", 0.15);
  global_timeout_ = number("global_map_timeout", 3.0);
  local_timeout_ = number("local_map_timeout", 1.0);
  base_uncertainty_ = number("model_error_margin", 0.025);
  max_tracking_error_ = number("max_tracking_error", 0.5);
  smoother_.spacing = number("path_spacing", 0.04);
  smoother_.max_displacement = number("smoothing_max_displacement", 0.1);
  smoother_.iterations = node_->declare_parameter<int>(name + ".smoothing_iterations", 12);
  if (!valid_positive(cruise_speed_) || cruise_speed_ > config_.max_speed ||
      !valid_positive(lateral_acceleration_) || !valid_positive(braking_acceleration_) ||
      !valid_positive(goal_tolerance_) || !valid_positive(goal_speed_tolerance_) ||
      !std::isfinite(actuator_delay_) || actuator_delay_ < 0.0 || actuator_delay_ > 0.5 ||
      !valid_positive(global_timeout_) || !valid_positive(local_timeout_) ||
      !valid_positive(base_uncertainty_) || !valid_positive(max_tracking_error_) ||
      !valid_positive(smoother_.spacing) || !valid_positive(smoother_.max_displacement) ||
      smoother_.iterations < 0 || smoother_.iterations > 100) {
    throw std::invalid_argument("Invalid MPC tracking/trajectory parameters");
  }
  solver_.reset(new MpcSolver(config_));
  if (!solver_->valid()) { throw std::invalid_argument("Invalid MPC solver parameters"); }
  const auto global_topic = node_->declare_parameter<std::string>(name + ".costmap_topic", "/global_costmap");
  const auto local_topic = node_->declare_parameter<std::string>(name + ".local_costmap_topic", "/local_costmap/costmap");
  global_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      global_topic, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        std::lock_guard<std::mutex> lock(mutex_);
        global_map_ = std::move(map); global_received_ = node_->now(); dirty_ = true;
      });
  local_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_topic, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        std::lock_guard<std::mutex> lock(mutex_);
        local_map_ = std::move(map); local_received_ = node_->now(); dirty_ = true;
      });
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", rclcpp::QoS(1), [this](geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        exact_goal_ = std::move(goal); dirty_ = true;
      });
  smooth_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/mpc/smoothed_path", rclcpp::QoS(1).transient_local());
  predicted_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/mpc/predicted_path", 1);
  metrics_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/mpc/metrics", 1);
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  raw_path_ = path;
  dirty_ = true;
  if (path.poses.empty()) { path_.clear(); arc_.clear(); speeds_.clear(); }
}

bool MpcController::preparePath(const SafetyGrid & grid)
{
  path_.clear(); arc_.clear(); speeds_.clear();
  path_frame_ = raw_path_.header.frame_id;
  if (!grid.ready() || grid.frame() != path_frame_ || raw_path_.poses.empty() ||
      raw_path_.poses.size() > 10000) { return false; }
  std::vector<Point> raw;
  for (const auto & pose : raw_path_.poses) {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != path_frame_) ||
        !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) { return false; }
    raw.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  // A* ends at a cell center. Restore the clicked goal only within that cell's vicinity.
  if (exact_goal_ && exact_goal_->header.frame_id == path_frame_) {
    const Point goal(exact_goal_->pose.position.x, exact_goal_->pose.position.y);
    if (goal.allFinite() && (goal - raw.back()).norm() <= 0.10 &&
        grid.segmentFree(raw.back(), goal)) { raw.push_back(goal); }
  }
  path_ = TrajectoryOptimizer::optimize(raw, grid, smoother_);
  if (path_.empty()) { return false; }
  arc_.resize(path_.size(), 0.0);
  for (std::size_t i = 1; i < path_.size(); ++i) { arc_[i] = arc_[i-1] + (path_[i]-path_[i-1]).norm(); }
  speeds_.assign(path_.size(), cruise_speed_);
  for (std::size_t i = 0; i < path_.size(); ++i) {
    // Estimate curvature over a physical window instead of adjacent tiny grid edges.
    const Point back = sample(std::max(0.0, arc_[i]-0.15)).position;
    const Point front = sample(std::min(arc_.back(), arc_[i]+0.15)).position;
    const Point a = path_[i] - back, b = front - path_[i];
    if (a.norm() > 1e-5 && b.norm() > 1e-5) {
      const double angle = std::acos(std::max(-1.0, std::min(1.0, a.dot(b)/(a.norm()*b.norm()))));
      const double curvature = angle / std::max(0.05, 0.5*(a.norm()+b.norm()));
      speeds_[i] = std::min(speeds_[i], std::sqrt(lateral_acceleration_/std::max(0.01, curvature)));
    }
    speeds_[i] *= std::max(0.4, 1.0 - 0.006*grid.cost(path_[i]));
  }
  speeds_.back() = 0.0;
  // Backward braking pass and forward acceleration pass form a feasible speed profile.
  for (std::size_t i = path_.size()-1; i > 0; --i) {
    speeds_[i-1] = std::min(speeds_[i-1],
        std::sqrt(speeds_[i]*speeds_[i] + 2.0*braking_acceleration_*(arc_[i]-arc_[i-1])));
  }
  for (std::size_t i = 1; i < path_.size(); ++i) {
    speeds_[i] = std::min(speeds_[i],
        std::sqrt(speeds_[i-1]*speeds_[i-1] + 2.0*braking_acceleration_*(arc_[i]-arc_[i-1])));
  }
  dirty_ = false;
  return true;
}

MpcController::Projection MpcController::project(const Point & position) const
{
  Projection result;
  result.distance = std::numeric_limits<double>::infinity();
  if (path_.size() == 1) { return {0.0, (position-path_[0]).norm()}; }
  for (std::size_t i = 1; i < path_.size(); ++i) {
    const Point delta = path_[i]-path_[i-1];
    const double length2 = delta.squaredNorm();
    const double t = length2 > 1e-12 ? std::max(0.0, std::min(1.0, (position-path_[i-1]).dot(delta)/length2)) : 0.0;
    const double distance = (position-path_[i-1]-t*delta).norm();
    if (distance < result.distance) { result = {arc_[i-1]+t*std::sqrt(length2), distance}; }
  }
  return result;
}

Reference MpcController::sample(double arc) const
{
  Reference ref;
  if (path_.empty()) { return ref; }
  if (path_.size() == 1 || arc >= arc_.back()) { ref.position = path_.back(); return ref; }
  arc = std::max(0.0, arc);
  const auto end = std::upper_bound(arc_.begin(), arc_.end(), arc);
  const auto i = std::min(path_.size()-1, static_cast<std::size_t>(end-arc_.begin()));
  const double length = arc_[i]-arc_[i-1];
  const Point tangent = length > 1e-10 ? Point((path_[i]-path_[i-1])/length) : Point::Zero();
  ref.position = path_[i-1] + (arc-arc_[i-1])*tangent;
  if (!speeds_.empty()) {
    const double ratio = length > 1e-10 ? (arc-arc_[i-1])/length : 0.0;
    ref.velocity = ((1.0-ratio)*speeds_[i-1]+ratio*speeds_[i])*tangent;
  }
  return ref;
}
double MpcController::speedAt(double arc) const { return sample(arc).velocity.norm(); }

bool MpcController::sweptSafe(const State & initial, const std::vector<Point> & controls,
                             const SafetyGrid & grid, double uncertainty) const
{
  State state = initial;
  auto move = [&](const Point & u, double duration, double radius) {
    const int steps = std::max(1, static_cast<int>(std::ceil(duration/0.025)));
    for (int step = 0; step < steps; ++step) {
      const State next = MpcSolver::step(state, u, duration/steps, config_.tau);
      if (!grid.segmentFree(state.position, next.position)) { return false; }
      // This is a model-error envelope, not a second robot-footprint inflation.
      for (int j = 0; j < 8; ++j) {
        const double angle = j*std::acos(-1.0)/4.0;
        const Point offset(radius*std::cos(angle), radius*std::sin(angle));
        if (!grid.segmentFree(state.position+offset, next.position+offset)) { return false; }
      }
      state = next;
    }
    return true;
  };
  if (actuator_delay_ > 0.0 && !move(previous_command_, actuator_delay_, uncertainty)) { return false; }
  for (std::size_t k = 0; k < controls.size(); ++k) {
    const double radius = uncertainty + std::min(0.06, model_error_*config_.dt*(k+1));
    if (!move(controls[k], config_.dt, radius)) { return false; }
  }
  // Check a zero-command tail, including residual velocity beyond the MPC horizon.
  const int tail = static_cast<int>(std::ceil(4.0*config_.tau/config_.dt));
  for (int k = 0; k < tail; ++k) {
    if (!move(Point::Zero(), config_.dt, uncertainty+0.06)) { return false; }
  }
  return true;
}

void MpcController::publishPaths(const rclcpp::Time & stamp, const std::string & frame,
                                const std::vector<State> & predicted)
{
  if (visualization_stamp_ > 0 && stamp.nanoseconds() >= visualization_stamp_ &&
      stamp.nanoseconds()-visualization_stamp_ < 200000000) { return; }
  visualization_stamp_ = stamp.nanoseconds();
  auto make = [&](const std::vector<Point> & points) {
    nav_msgs::msg::Path message;
    message.header.stamp = stamp; message.header.frame_id = frame;
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header; pose.pose.position.x = point.x(); pose.pose.position.y = point.y();
      pose.pose.orientation.w = 1.0; message.poses.push_back(pose);
    }
    return message;
  };
  if (dirty_) { smooth_pub_->publish(make({})); }
  else { smooth_pub_->publish(make(path_)); }
  std::vector<Point> positions;
  for (const auto & state : predicted) { positions.push_back(state.position); }
  predicted_pub_->publish(make(positions));
}

geometry_msgs::msg::TwistStamped MpcController::stop(const rclcpp::Time & now, const char * reason)
{
  previous_command_.setZero(); command_stamp_ = now.nanoseconds();
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now; command.header.frame_id = base_frame_;
  nav_msgs::msg::Path empty;
  empty.header.stamp = now; empty.header.frame_id = path_frame_;
  predicted_pub_->publish(empty);
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "MPC holding zero: %s", reason);
  return command;
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_->now();
  const rclcpp::Time measured(pose.header.stamp, now.get_clock_type());
  const double age = (now-measured).seconds();
  const Point position(pose.pose.position.x, pose.pose.position.y);
  const Point speed(velocity.linear.x, velocity.linear.y);
  const auto & q = pose.pose.orientation;
  const double qnorm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (measured.nanoseconds() <= 0 || !std::isfinite(age) || age < 0.0 ||
      age >= kPredictionStopAge || !position.allFinite() ||
      !speed.allFinite() || !std::isfinite(qnorm) || qnorm < 1e-12 ||
      pose.header.frame_id != raw_path_.header.frame_id) { return stop(now, "invalid/stale feedback"); }
  const double global_age = global_map_ ? (now-global_received_).seconds() : -1.0;
  const double local_age = local_map_ ? (now-local_received_).seconds() : -1.0;
  if (global_age < 0.0 || global_age > global_timeout_ ||
      local_age < 0.0 || local_age > local_timeout_) { return stop(now, "missing/stale costmap"); }
  // Header age catches queued old maps even when their receive time is recent.
  for (const auto & map : {global_map_, local_map_}) {
    const rclcpp::Time stamp(map->header.stamp, now.get_clock_type());
    const double map_age = (now-stamp).seconds();
    const double timeout = map == global_map_ ? global_timeout_ : local_timeout_;
    if (stamp.nanoseconds() <= 0 || map_age < 0.0 || map_age > timeout) { return stop(now, "old map source stamp"); }
  }
  SafetyGrid grid(*global_map_, local_map_.get());
  if (!grid.ready() || grid.frame() != pose.header.frame_id) { return stop(now, "invalid map frame/data"); }
  if (dirty_ && !preparePath(grid)) { return stop(now, "no validated trajectory"); }
  if (path_.empty()) { return stop(now, "empty trajectory"); }
  tf2::Quaternion rotation(q.x, q.y, q.z, q.w); rotation.normalize();
  const double yaw = tf2::getYaw(rotation);
  const std::int64_t stamp = measured.nanoseconds();
  if (measurement_stamp_ > 0 && stamp > measurement_stamp_) {
    const double dt = (stamp-measurement_stamp_)*1e-9;
    if (dt >= 0.001 && dt <= 0.5) {
      const double difference = std::atan2(std::sin(yaw-previous_yaw_), std::cos(yaw-previous_yaw_));
      yaw_rate_ = std::max(-3.0, std::min(3.0, difference/dt));
      if (dt <= 0.2) {
        const double decay = std::exp(-dt/config_.tau);
        const Point expected = decay*previous_measured_velocity_+(1.0-decay)*previous_command_;
        model_error_ = 0.9*model_error_+0.1*std::min(0.15, (speed-expected).norm());
      }
    }
  } else if (stamp < measurement_stamp_) { yaw_rate_ = 0.0; model_error_ = 0.0; }
  if (stamp != measurement_stamp_) {
    measurement_stamp_ = stamp; previous_yaw_ = yaw; previous_measured_velocity_ = speed;
  }
  State current; current.position = position + age*speed; current.velocity = speed;
  const double yaw_now = yaw + yaw_rate_*age;
  const double goal_distance = (path_.back()-current.position).norm();
  if (goal_distance <= goal_tolerance_ && speed.norm() <= goal_speed_tolerance_) {
    settled_ = true;
    return stop(now, "goal settled");
  }
  settled_ = false;
  const auto nearest = project(current.position);
  if (nearest.distance > max_tracking_error_) { return stop(now, "tracking error exceeds recovery corridor"); }
  State delayed = current;
  if (actuator_delay_ > 0.0) { delayed = MpcSolver::step(current, previous_command_, actuator_delay_, config_.tau); }
  const double uncertainty = base_uncertainty_ + std::min(0.05, model_error_) + config_.max_speed*age;
  const double feedback_scale = age <= kPredictionFullSpeedAge ? 1.0 :
      (kPredictionStopAge-age)/
      (kPredictionStopAge-kPredictionFullSpeedAge);
  const double error_scale = 1.0/(1.0+5.0*model_error_+2.0*nearest.distance);
  const double elapsed = command_stamp_ > 0 ? (now.nanoseconds()-command_stamp_)*1e-9 : 0.02;
  const double command_dt = std::max(0.001, std::min(0.1, elapsed));
  const auto started = std::chrono::steady_clock::now();
  SolverResult selected;
  Point output = Point::Zero();
  for (double trial : {1.0, 0.6, 0.3}) {
    std::vector<Reference> refs;
    double s = project(delayed.position).arc;
    for (int k = 0; k < config_.horizon; ++k) {
      const double rate = speedAt(s)*trial*feedback_scale*error_scale;
      s = std::min(arc_.back(), s+config_.dt*rate);
      auto ref = sample(s);
      ref.velocity *= trial*feedback_scale*error_scale;
      // Terminal approach targets the actual endpoint rather than stalling at zero profile speed.
      if (arc_.back()-nearest.arc < 0.20) { ref.position = path_.back(); ref.velocity.setZero(); }
      refs.push_back(ref);
    }
    auto result = solver_->solve(delayed, refs, previous_command_);
    if (!result.success) { continue; }
    Point candidate = previous_command_ + limit(result.controls.front()-previous_command_,
                                                config_.max_acceleration*command_dt);
    candidate = limit(candidate, config_.max_speed*feedback_scale);
    // Validate both the optimizer's sequence and the command actually emitted at 50 Hz.
    auto actual = result.controls;
    actual[0] = candidate;
    if (!sweptSafe(current, result.controls, grid, uncertainty) ||
        !sweptSafe(current, actual, grid, uncertainty)) { continue; }
    selected = std::move(result); output = candidate; break;
  }
  if (!selected.success) { return stop(now, "no collision-checked MPC candidate"); }
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now; command.header.frame_id = base_frame_;
  command.twist.linear.x = std::cos(yaw_now)*output.x()+std::sin(yaw_now)*output.y();
  command.twist.linear.y = -std::sin(yaw_now)*output.x()+std::cos(yaw_now)*output.y();
  previous_command_ = output; command_stamp_ = now.nanoseconds();
  const double solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-started).count();
  // Fields: path error, goal error, measured speed, commanded speed, solve ms,
  // odom age, model velocity residual, iterations, solver convergence flag.
  std_msgs::msg::Float64MultiArray metrics;
  metrics.data = {nearest.distance, goal_distance, speed.norm(), output.norm(), solve_ms,
                  age, model_error_, static_cast<double>(selected.iterations), selected.converged ? 1.0 : 0.0};
  metrics_pub_->publish(metrics);
  // Limit visualization traffic; no per-cycle INFO logging on this VM.
  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "MPC error=%.3f goal=%.3f speed=%.3f solve=%.2fms", nearest.distance, goal_distance, output.norm(), solve_ms);
  static_cast<void>(settled_);
  publishPaths(now, pose.header.frame_id, selected.states);
  return command;
}
}  // namespace mpc_controller

PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcController, sp_controller_server::ControllerPlugin)
