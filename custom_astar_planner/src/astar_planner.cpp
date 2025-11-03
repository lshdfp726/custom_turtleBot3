#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <cmath>
#include <queue>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"

// 显式指定消息类型的完整命名空间
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using namespace nav2_core;

namespace custom_astar_planner
{
struct Node
{
  int x, y;
  double g_cost, h_cost, f_cost;
  Node* parent;
  bool operator<(const Node& other) const { return f_cost > other.f_cost; }
};

class AStarCore
{
public:
  // 构造函数直接接收节点 WeakPtr，并用节点日志器初始化 logger_
  explicit AStarCore(rclcpp::Logger logger)
    : logger_(std::move(logger)), // 存储节点指针（供后续使用）
      costmap_(nullptr),
      start_x_(0), start_y_(0), goal_x_(0), goal_y_(0),
      allow_unknown_(false),
      w_euc_cost_(1.0)
  {}
  ~AStarCore() = default;


  void setCostmap(nav2_costmap_2d::Costmap2D* costmap) { costmap_ = costmap; }
  nav2_costmap_2d::Costmap2D* getCostmap() { return costmap_; }
  
  bool setStartAndGoal(const PoseStamped& start, const PoseStamped& goal)
  {
    if (!costmap_) return false;

    unsigned int mx, my;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my))
      return false;
    start_x_ = static_cast<int>(mx);
    start_y_ = static_cast<int>(my);

    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my))
      return false;
    goal_x_ = static_cast<int>(mx);
    goal_y_ = static_cast<int>(my);

    if (!isValid(start_x_, start_y_) || !isValid(goal_x_, goal_y_))
      return false;

    if (costmap_->getCost(static_cast<unsigned int>(goal_x_), static_cast<unsigned int>(goal_y_)) == nav2_costmap_2d::LETHAL_OBSTACLE)
      return false;

    return true;
  }

  bool generatePath(std::vector<PoseStamped>& path, std::function<bool()> cancel_checker)
  {
    if (start_x_ == goal_x_ && start_y_ == goal_y_) return false;
    RCLCPP_INFO(logger_, "A* Planner generatePath");
    std::priority_queue<Node*> open_list;
    std::vector<std::vector<Node*>> grid(
      costmap_->getSizeInCellsX(), 
      std::vector<Node*>(costmap_->getSizeInCellsY(), nullptr)
    );

    Node* start_node = new Node{start_x_, start_y_, 0.0, heuristic(start_x_, start_y_), 0.0, nullptr};
    start_node->f_cost = start_node->g_cost + start_node->h_cost;
    open_list.push(start_node);
    grid[static_cast<unsigned int>(start_x_)][static_cast<unsigned int>(start_y_)] = start_node;

    const int dirs[8][2] = {{-1, -1}, {-1, 0}, {-1, 1},
                            {0, -1},          {0, 1},
                            {1, -1},  {1, 0}, {1, 1}};
    double diagonal_cost = sqrt(2.0);

    while (!open_list.empty() && !cancel_checker()) {
      Node* current = open_list.top();
      open_list.pop();

      if (current->x == goal_x_ && current->y == goal_y_) {
        reconstructPath(current, path);
        for (auto& row : grid)
          for (auto& node : row)
            delete node;
        return true;
      }

      for (auto& dir : dirs) {
        int nx = current->x + dir[0];
        int ny = current->y + dir[1];

        if (!isValid(nx, ny) || grid[static_cast<unsigned int>(nx)][static_cast<unsigned int>(ny)] != nullptr)
          continue;

        double move_cost = (dir[0] != 0 && dir[1] != 0) ? diagonal_cost : 1.0;
        double new_g = current->g_cost + move_cost * (costmap_->getCost(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)) / 100.0 + 1.0);

        Node* neighbor = new Node{nx, ny, new_g, heuristic(nx, ny), 0.0, current};
        neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
        open_list.push(neighbor);
        grid[static_cast<unsigned int>(nx)][static_cast<unsigned int>(ny)] = neighbor;
      }
    }

    for (auto& row : grid)
      for (auto& node : row)
        delete node;
    return false;
  }

  void setParams(bool allow_unknown, double w_euc_cost)
  {
    allow_unknown_ = allow_unknown;
    w_euc_cost_ = w_euc_cost;
  }

private:
  rclcpp::Logger logger_;
  double heuristic(int x, int y)
  {
    double dx = static_cast<double>(std::abs(x - goal_x_));
    double dy = static_cast<double>(std::abs(y - goal_y_));
    return sqrt(dx*dx + dy*dy) * w_euc_cost_;
  }

  bool isValid(int x, int y)
  {
    return x >= 0 && x < static_cast<int>(costmap_->getSizeInCellsX()) &&
           y >= 0 && y < static_cast<int>(costmap_->getSizeInCellsY()) &&
           costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) != nav2_costmap_2d::LETHAL_OBSTACLE &&
           (allow_unknown_ || costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) != nav2_costmap_2d::NO_INFORMATION);
  }

  void reconstructPath(Node* end_node, std::vector<PoseStamped>& path)
  {
    Node* current = end_node;
    while (current != nullptr) {
      PoseStamped pose;
      costmap_->mapToWorld(static_cast<unsigned int>(current->x), static_cast<unsigned int>(current->y), 
                          pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
      path.push_back(pose);
      current = current->parent;
    }
    std::reverse(path.begin(), path.end());
  }

  nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
  int start_x_, start_y_, goal_x_, goal_y_;
  bool allow_unknown_ = false;
  double w_euc_cost_ = 1.0;
};

class AStarPlanner : public GlobalPlanner
{
public:
  AStarPlanner() 
    : logger_(rclcpp::get_logger("AStarPlanner")),
      allow_unknown_(false),
      w_euc_cost_(1.0),
      interpolation_dist_(0.05)
  {}
  ~AStarPlanner() override = default;

  // 严格匹配父类的configure函数签名
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    parent_node_ = parent;
    auto node = parent_node_.lock();
    logger_ = node->get_logger();

    astar_core_ = std::make_unique<AStarCore>(logger_);
    astar_core_->setCostmap(costmap_ros->getCostmap());

    clock_ = node->get_clock();
    name_ = name;
    tf_ = tf;
    global_frame_ = costmap_ros->getGlobalFrameID();

    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".w_euc_cost", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".interpolation_distance", rclcpp::ParameterValue(0.05));

    node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
    node->get_parameter(name_ + ".w_euc_cost", w_euc_cost_);
    node->get_parameter(name_ + ".interpolation_distance", interpolation_dist_);

    astar_core_->setParams(allow_unknown_, w_euc_cost_);
    RCLCPP_INFO(logger_, "A* Planner configured successfully");
  }

  void cleanup() override
  {
    RCLCPP_INFO(logger_, "A* Planner cleaning up");
    astar_core_.reset();
  }

  void activate() override
  {
    RCLCPP_INFO(logger_, "A* Planner activated");
    auto node = parent_node_.lock();
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&AStarPlanner::dynamicParamsCallback, this, std::placeholders::_1));
  }

  void deactivate() override
  {
    RCLCPP_INFO(logger_, "A* Planner deactivated");
    auto node = parent_node_.lock();
    if (node && dyn_params_handler_) {
      node->remove_on_set_parameters_callback(dyn_params_handler_.get());
    }
    dyn_params_handler_.reset();
  }

  // 修复：只实现父类要求的单参数createPlan（匹配Nav2旧版本接口）
  Path createPlan(
    const PoseStamped & start,
    const PoseStamped & goal) override
  {
    // 内部调用带cancel_checker的实现，默认不取消
    RCLCPP_INFO(logger_, "A* Planner createPlan");
    return createPlanWithChecker(start, goal, []() { return false; });
  }

private:
  // 内部实现带cancel_checker的版本，避免与父类签名冲突
  Path createPlanWithChecker(
    const PoseStamped & start,
    const PoseStamped & goal,
    std::function<bool()> cancel_checker)
  {
    Path global_path;
    auto start_time = std::chrono::steady_clock::now();

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(astar_core_->getCostmap()->getMutex()));

    unsigned int mx_start, my_start, mx_goal, my_goal;
    if (!astar_core_->getCostmap()->worldToMap(
          start.pose.position.x, start.pose.position.y, mx_start, my_start))
    {
      throw std::runtime_error("Start position is outside map bounds");
    }

    if (!astar_core_->getCostmap()->worldToMap(
          goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal))
    {
      throw std::runtime_error("Goal position is outside map bounds");
    }

    RCLCPP_INFO(logger_, "begin A* mx_start == mx_goal && my_start == my_goal");
    if (mx_start == mx_goal && my_start == my_goal) {
      global_path.header.stamp = clock_->now();
      global_path.header.frame_id = global_frame_;
      PoseStamped pose = start;
      pose.pose.orientation = goal.pose.orientation;
      global_path.poses.push_back(pose);
      return global_path;
    }

    if (!astar_core_->setStartAndGoal(start, goal)) {
      throw std::runtime_error("Start or goal is invalid (obstacle or unknown)");
    }

    std::vector<PoseStamped> raw_path;
    RCLCPP_INFO(logger_, "begin A* Planner generatePath");
    if (!astar_core_->generatePath(raw_path, cancel_checker)) {
      throw std::runtime_error("No valid A* path found");
    }

    RCLCPP_INFO(logger_, "begin A* linearInterpolation");
    global_path = linearInterpolation(raw_path, interpolation_dist_);
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    if (!global_path.poses.empty()) {
      global_path.poses.back().pose.orientation = goal.pose.orientation;
    }

    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time);
    RCLCPP_DEBUG(logger_, "A* Planning time: %ld ms, Waypoints: %zu",
                dur.count(), global_path.poses.size());

    return global_path;
  }

  Path linearInterpolation(
    const std::vector<PoseStamped>& raw_path,
    const double& dist_bw_points)
  {
    Path smoothed_path;
    if (raw_path.empty()) return smoothed_path;
    RCLCPP_INFO(logger_, "A* linearInterpolation");
    smoothed_path.header = raw_path[0].header;

    for (size_t i = 0; i < raw_path.size() - 1; ++i) {
      const auto& p1 = raw_path[i].pose.position;
      const auto& p2 = raw_path[i+1].pose.position;

      double distance = std::hypot(p2.x - p1.x, p2.y - p1.y);
      if (distance < 1e-6) continue;

      int loops = static_cast<int>(distance / dist_bw_points);
      double cos_theta = (p2.x - p1.x) / distance;
      double sin_theta = (p2.y - p1.y) / distance;

      for (int k = 0; k <= loops; ++k) {
        PoseStamped pose;
        pose.header = smoothed_path.header;
        pose.pose.position.x = p1.x + k * dist_bw_points * cos_theta;
        pose.pose.position.y = p1.y + k * dist_bw_points * sin_theta;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
          atan2(sin_theta, cos_theta));
        smoothed_path.poses.push_back(pose);
      }
    }

    if (!raw_path.empty()) {
      smoothed_path.poses.back() = raw_path.back();
    }

    return smoothed_path;
  }

  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : parameters) {
      if (param.get_name() == name_ + ".allow_unknown") {
        allow_unknown_ = param.as_bool();
        astar_core_->setParams(allow_unknown_, w_euc_cost_);
      } else if (param.get_name() == name_ + ".w_euc_cost") {
        w_euc_cost_ = param.as_double();
        astar_core_->setParams(allow_unknown_, w_euc_cost_);
      }
    }

    return result;
  }

  std::unique_ptr<AStarCore> astar_core_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_, global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  bool allow_unknown_;
  double w_euc_cost_;
  double interpolation_dist_;
};

}  // namespace custom_astar_planner

PLUGINLIB_EXPORT_CLASS(custom_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)