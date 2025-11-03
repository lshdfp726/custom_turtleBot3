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
#include <tf2/utils.hpp>
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
      w_euc_cost_(1.0),
      map_resolution_(0.05) // 地图默认分辨率（0.05m/格子，可从costmap获取
  {}
  ~AStarCore() = default;

  void setRobotRadius(double radius) { robot_radius_ = radius; }
  void setCostmap(nav2_costmap_2d::Costmap2D* costmap) { costmap_ = costmap; }
  nav2_costmap_2d::Costmap2D* getCostmap() { return costmap_; }

  void setMapResolution() {
    if (costmap_) {
      map_resolution_ = costmap_->getResolution();
      RCLCPP_INFO(logger_, "地图分辨率：%.3fm/格子", map_resolution_);
    }
  }
  bool setStartAndGoal(const PoseStamped& start, const PoseStamped& goal)
  {
    // 1. 检查代价地图是否有效
    if (!costmap_) {
      RCLCPP_ERROR(logger_, "setStartAndGoal 失败：代价地图为空！");
      return false;
    }

    unsigned int mx, my;

    // 2. 转换起点坐标（世界坐标 → 地图格子坐标）
    bool start_ok = costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, mx, my);
    if (!start_ok) {
      RCLCPP_ERROR(logger_, "起点坐标转换失败！世界坐标：(%.2f, %.2f)，超出地图范围",
                  start.pose.position.x, start.pose.position.y);
      return false;
    }
    start_x_ = static_cast<int>(mx);
    start_y_ = static_cast<int>(my);
    RCLCPP_INFO(logger_, "起点转换成功：世界坐标(%.2f, %.2f) → 地图格子(%d, %d)",
                start.pose.position.x, start.pose.position.y, start_x_, start_y_);

    // 3. 转换终点坐标（世界坐标 → 地图格子坐标）
    bool goal_ok = costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, mx, my);
    if (!goal_ok) {
      RCLCPP_ERROR(logger_, "终点坐标转换失败！世界坐标：(%.2f, %.2f)，超出地图范围",
                  goal.pose.position.x, goal.pose.position.y);
      return false;
    }
    goal_x_ = static_cast<int>(mx);
    goal_y_ = static_cast<int>(my);
    RCLCPP_INFO(logger_, "终点转换成功：世界坐标(%.2f, %.2f) → 地图格子(%d, %d)",
                goal.pose.position.x, goal.pose.position.y, goal_x_, goal_y_);

    // 4. 验证起点是否有效（非障碍物、在地图内）
    if (!isValid(start_x_, start_y_)) {
      unsigned char start_cost = costmap_->getCost(
        static_cast<unsigned int>(start_x_), static_cast<unsigned int>(start_y_));
      RCLCPP_ERROR(logger_, "起点无效！格子坐标(%d, %d)，代价=%d（可能是障碍物或未知区域）",
                  start_x_, start_y_, start_cost);
      return false;
    }

    // 5. 验证终点是否有效（非障碍物）
    unsigned char goal_cost = costmap_->getCost(
      static_cast<unsigned int>(goal_x_), static_cast<unsigned int>(goal_y_));
    if (goal_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_ERROR(logger_, "终点是障碍物！格子坐标(%d, %d)，代价=%d（致命障碍）",
                  goal_x_, goal_y_, goal_cost);
      return false;
    }
    if (!isValid(goal_x_, goal_y_)) {
      RCLCPP_ERROR(logger_, "终点无效！格子坐标(%d, %d)，代价=%d（可能是未知区域且不允许通过）",
                  goal_x_, goal_y_, goal_cost);
      return false;
    }

    // 6. 验证起点和终点是否相同
    if (start_x_ == goal_x_ && start_y_ == goal_y_) {
      RCLCPP_WARN(logger_, "起点和终点相同！格子坐标(%d, %d)", start_x_, start_y_);
      return false; // 可选：返回true并生成空路径，根据需求调整
    }

    RCLCPP_INFO(logger_, "起点和终点验证通过，可开始规划");
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
  double robot_radius_ = 0.22; // 默认 TurtleBot3 半径
  nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
  int start_x_, start_y_, goal_x_, goal_y_;
  bool allow_unknown_ = false;
  double w_euc_cost_ = 1.0;
  double map_resolution_ = 0.05;
  
  double heuristic(int x, int y)
  {
    double dx = static_cast<double>(std::abs(x - goal_x_));
    double dy = static_cast<double>(std::abs(y - goal_y_));
    return sqrt(dx*dx + dy*dy) * w_euc_cost_;
  }

  bool isValid(int x, int y)
  {
    // 1. 先判断当前格子是否在地图内
    if (x < 0 || x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
        y < 0 || y >= static_cast<int>(costmap_->getSizeInCellsY())) {
      return false;
    }

    unsigned int mx = static_cast<unsigned int>(x);
    unsigned int my = static_cast<unsigned int>(y);
    unsigned char cost = costmap_->getCost(mx, my);

    // 2. 原逻辑：不是障碍物 + 允许未知区域
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION)) {
      return false;
    }

    // 3. 新增：判断当前格子周围 N 个格子是否有障碍物（N = 安全格子半径）
    int safe_radius = getSafeCellRadius();
    for (int dx = -safe_radius; dx <= safe_radius; ++dx) {
      for (int dy = -safe_radius; dy <= safe_radius; ++dy) {
        // 计算周围格子坐标
        int nx = x + dx;
        int ny = y + dy;
        // 周围格子必须在地图内
        if (nx < 0 || nx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
            ny < 0 || ny >= static_cast<int>(costmap_->getSizeInCellsY())) {
          continue;
        }
        unsigned int nmx = static_cast<unsigned int>(nx);
        unsigned int nmy = static_cast<unsigned int>(ny);
        unsigned char neighbor_cost = costmap_->getCost(nmx, nmy);
        // 若周围格子是障碍物 → 当前格子不安全，返回false
        if (neighbor_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          return false;
        }
      }
    }

    // 4. 所有条件满足 → 格子有效
    return true;
  }

  void reconstructPath(Node* end_node, std::vector<PoseStamped>& path) {
    Node* current = end_node;
    std::vector<PoseStamped> temp_path;
    while (current != nullptr) {
      PoseStamped pose;
      costmap_->mapToWorld(static_cast<unsigned int>(current->x), static_cast<unsigned int>(current->y), 
                          pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
      pose.header.frame_id = "map"; // 明确坐标系
      temp_path.push_back(pose);
      current = current->parent;
    }
    std::reverse(temp_path.begin(), temp_path.end());

    // 关键：稀疏化原始路径（每隔5个点保留1个，可调整）
    path.clear();
    const int sparse_step = 5; // 步长越大，点数越少
    for (size_t i = 0; i < temp_path.size(); i += sparse_step) {
      path.push_back(temp_path[i]);
    }
    // 确保保留最后一个点（终点）
    if (!temp_path.empty() && !path.empty() && path.back() != temp_path.back()) {
      path.push_back(temp_path.back());
    }

    RCLCPP_INFO(logger_, "稀疏化后原始路径点数：%zu", path.size());
  }
  int getSafeCellRadius() const {
    return static_cast<int>(ceil(robot_radius_ / map_resolution_));
  }
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
    RCLCPP_WARN(logger_, "global_frame_ is ：%s", global_frame_.c_str());
      // 新增：强制坐标系为 map（TurtleBot3 默认全局帧）
    if (global_frame_ != "map") {
      RCLCPP_WARN(logger_, "强制将坐标系改为 map，原坐标系：%s", global_frame_.c_str());
      global_frame_ = "map";
    }

    double robot_radius = 0.22;
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
    astar_core_->setRobotRadius(robot_radius);
    astar_core_->setMapResolution();
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

    // 新增：输出原始路径信息
    RCLCPP_INFO(logger_, "A* 原始路径点数：%zu", raw_path.size());
    for (size_t i = 0; i < raw_path.size(); ++i) {
      RCLCPP_DEBUG(logger_, "原始路径点%ld：(%.2f, %.2f)",
                  i, raw_path[i].pose.position.x, raw_path[i].pose.position.y);
    }


    RCLCPP_INFO(logger_, "begin A* linearInterpolation");
    global_path = linearInterpolation(raw_path, interpolation_dist_);
    RCLCPP_INFO(logger_, "插值后路径点数：%zu，坐标系：%s",
            global_path.poses.size(), global_path.header.frame_id.c_str());
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

    // 遍历原始路径点，生成平滑路径
    for (size_t i = 0; i < raw_path.size() - 1; ++i) {
      const auto& p1 = raw_path[i].pose.position;
      const auto& p2 = raw_path[i+1].pose.position;

      double distance = std::hypot(p2.x - p1.x, p2.y - p1.y);
      if (distance < 1e-6) continue;

      int loops = static_cast<int>(distance / dist_bw_points);
      double cos_theta = (p2.x - p1.x) / distance;
      double sin_theta = (p2.y - p1.y) / distance;
      // 目标角度（p1→p2的方向）
      double target_angle = atan2(sin_theta, cos_theta);

      // 生成当前段的插值点
      for (int k = 0; k <= loops; ++k) {
        PoseStamped pose;
        pose.header = smoothed_path.header;
        // 位置插值（原有逻辑不变）
        pose.pose.position.x = p1.x + k * dist_bw_points * cos_theta;
        pose.pose.position.y = p1.y + k * dist_bw_points * sin_theta;
        pose.pose.position.z = 0.0;

        // 关键修改1：角度渐变平滑（避免尖锐拐角）
        if (smoothed_path.poses.empty()) {
          // 第一个点，直接用目标角度
          pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(target_angle);
        } else {
          // 非第一个点，获取上一个点的角度
          const auto& prev_pose = smoothed_path.poses.back();
          double prev_angle = tf2::getYaw(prev_pose.pose.orientation);

          // 计算角度差（限制在 [-π, π] 范围内）
          double angle_diff = std::fmod(target_angle - prev_angle + M_PI, 2 * M_PI) - M_PI;
          // 角度变化率限制（每次最多转 5 度，可调整）
          const double max_angle_step = 0.087; // 5° 转换为弧度（π/180*5）
          if (std::abs(angle_diff) > max_angle_step) {
            // 渐变转向，不超过最大步长
            pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
              prev_angle + (angle_diff > 0 ? max_angle_step : -max_angle_step));
          } else {
            // 角度差较小，直接用目标角度
            pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(target_angle);
          }
        }

        smoothed_path.poses.push_back(pose);
      }
    }

     Path deduped_path = smoothed_path;
    deduped_path.poses.clear();
    for (const auto& pose : smoothed_path.poses) {
      if (deduped_path.poses.empty()) {
        deduped_path.poses.push_back(pose);
      } else {
        const auto& last_pose = deduped_path.poses.back();
        double dist = std::hypot(
          pose.pose.position.x - last_pose.pose.position.x,
          pose.pose.position.y - last_pose.pose.position.y
        );
        if (dist > 0.01) { // 距离大于 1cm 才保留
          deduped_path.poses.push_back(pose);
        }
      }
    }
    smoothed_path = deduped_path;

    // 确保最后一个点与原始路径终点一致（原有逻辑不变）
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