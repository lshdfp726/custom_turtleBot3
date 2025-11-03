#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <cmath>
#include <queue>
#include <algorithm>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"

// 简化命名空间
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using namespace nav2_core;

namespace custom_astar_planner
{
// 极简节点结构
struct Node
{
  int x, y;                  // 地图格子坐标
  double g_cost, h_cost;     // g: 起点到当前代价, h: 启发式代价
  Node* parent;              // 父节点（路径重构用）

  // 优先队列排序：f_cost = g + h 越小越优先
  bool operator<(const Node& other) const {
    return (g_cost + h_cost) > (other.g_cost + other.h_cost);
  }
};

class AStarPlanner : public GlobalPlanner
{
public:
  AStarPlanner() 
    : logger_(rclcpp::get_logger("AStarPlanner")),
      costmap_(nullptr),
      allow_unknown_(false),
      map_resolution_(0.05) // TurtleBot3 地图默认分辨率
  {}
  ~AStarPlanner() override = default;

  // 配置函数：初始化代价地图、参数
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();
    costmap_ = costmap_ros->getCostmap(); // 获取代价地图
    global_frame_ = costmap_ros->getGlobalFrameID();
    clock_ = node->get_clock();
    name_ = name;

    // 强制坐标系为 map（TurtleBot3 全局帧）
    if (global_frame_ != "map") {
      RCLCPP_WARN(logger_, "强制坐标系为 map，原坐标系：%s", global_frame_.c_str());
      global_frame_ = "map";
    }

    // 声明并加载核心参数（极简版只保留必要参数）
    nav2_util::declare_parameter_if_not_declared(node, name_ + ".allow_unknown", false);
    node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

    RCLCPP_INFO(logger_, "极简 A* 规划器配置完成！");
  }

  void cleanup() override { RCLCPP_INFO(logger_, "规划器清理"); }
  void activate() override { RCLCPP_INFO(logger_, "规划器激活"); }
  void deactivate() override { RCLCPP_INFO(logger_, "规划器休眠"); }

  // 核心规划接口：生成路径（无插值，直接返回原始路径点）
  Path createPlan(
    const PoseStamped & start,
    const PoseStamped & goal) override
  {
    RCLCPP_INFO(logger_, "开始 A* 路径规划");
    Path global_path;
    global_path.header.frame_id = global_frame_;
    global_path.header.stamp = clock_->now();

    // 1. 检查代价地图是否有效
    if (!costmap_) {
      RCLCPP_ERROR(logger_, "代价地图为空！");
      return global_path;
    }

    // 2. 转换起点/终点（世界坐标 → 地图格子坐标）
    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
      RCLCPP_ERROR(logger_, "起点超出地图范围！");
      return global_path;
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
      RCLCPP_ERROR(logger_, "终点超出地图范围！");
      return global_path;
    }

    RCLCPP_INFO(logger_, "起点：世界坐标(%.2f, %.2f) → 格子(%u, %u)",
                start.pose.position.x, start.pose.position.y, start_x, start_y);
    RCLCPP_INFO(logger_, "终点：世界坐标(%.2f, %.2f) → 格子(%u, %u)",
                goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

    // 3. 执行 A* 核心搜索
    std::vector<PoseStamped> raw_path;
    if (!aStarSearch(static_cast<int>(start_x), static_cast<int>(start_y),
                     static_cast<int>(goal_x), static_cast<int>(goal_y), raw_path)) {
      RCLCPP_ERROR(logger_, "无有效路径！");
      return global_path;
    }

    // 4. 组装最终路径（直接用原始路径点，无任何处理）
    global_path.poses = raw_path;
    // 确保起点=机器人当前位置，终点=目标位置（避免搜索偏差）
    if (!global_path.poses.empty()) {
      global_path.poses.front() = start;
      global_path.poses.back() = goal;
    }

    RCLCPP_INFO(logger_, "规划完成！路径点数：%zu", global_path.poses.size());
    return global_path;
  }

private:
  // A* 核心搜索函数：输入格子坐标，输出世界坐标路径
  bool aStarSearch(int start_x, int start_y, int goal_x, int goal_y, std::vector<PoseStamped>& path)
  {
    // 方向数组：4方向（上下左右，简化搜索，减少路径点）
    const int dirs[4][2] = {{-1,0}, {1,0}, {0,-1}, {0,1}};
    const double move_cost = 1.0; // 4方向移动代价均为1

    // 初始化：开放列表（优先队列）、关闭列表（标记已处理）、节点网格
    std::priority_queue<Node*> open_list;
    std::vector<std::vector<bool>> closed_list(
      costmap_->getSizeInCellsX(), std::vector<bool>(costmap_->getSizeInCellsY(), false));
    std::vector<std::vector<Node*>> node_grid(
      costmap_->getSizeInCellsX(), std::vector<Node*>(costmap_->getSizeInCellsY(), nullptr));

    // 起点节点入队
    Node* start_node = new Node{start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y), nullptr};
    open_list.push(start_node);
    node_grid[start_x][start_y] = start_node;

    while (!open_list.empty()) {
      // 取出当前代价最小的节点
      Node* current = open_list.top();
      open_list.pop();

      // 到达终点：重构路径
      if (current->x == goal_x && current->y == goal_y) {
        reconstructPath(current, path);
        // 释放内存
        for (auto& row : node_grid)
          for (auto& node : row) delete node;
        return true;
      }

      // 标记为已处理
      closed_list[current->x][current->y] = true;

      // 遍历4个方向的邻居
      for (auto& dir : dirs) {
        int nx = current->x + dir[0];
        int ny = current->y + dir[1];

        // 检查邻居是否有效（在地图内 + 非障碍物 + 未处理）
        if (!isValid(nx, ny) || closed_list[nx][ny]) continue;

        // 计算新代价
        double new_g = current->g_cost + move_cost;

        // 邻居节点不存在 或 新代价更小 → 更新并加入开放列表
        if (!node_grid[nx][ny] || new_g < node_grid[nx][ny]->g_cost) {
          if (node_grid[nx][ny]) {
            node_grid[nx][ny]->g_cost = new_g;
            node_grid[nx][ny]->parent = current;
          } else {
            double new_h = heuristic(nx, ny, goal_x, goal_y);
            Node* neighbor = new Node{nx, ny, new_g, new_h, current};
            node_grid[nx][ny] = neighbor;
            open_list.push(neighbor);
          }
        }
      }
    }

    // 无路径：释放内存
    for (auto& row : node_grid)
      for (auto& node : row) delete node;
    return false;
  }

  // 启发式函数：欧氏距离（简化版用最简启发式）
  double heuristic(int x, int y, int goal_x, int goal_y) const
  {
    double dx = static_cast<double>(x - goal_x);
    double dy = static_cast<double>(y - goal_y);
    return std::sqrt(dx*dx + dy*dy);
  }

  // 节点有效性检查：地图内 + 非致命障碍 + 允许未知区域
  bool isValid(int x, int y) const
  {
    // 1. 检查是否在地图内
    if (x < 0 || x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
        y < 0 || y >= static_cast<int>(costmap_->getSizeInCellsY())) {
      return false;
    }

    // 2. 检查代价（只排除致命障碍，允许膨胀区域/未知区域）
    unsigned char cost = costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
    return cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
           (allow_unknown_ || cost != nav2_costmap_2d::NO_INFORMATION);
  }

  // 路径重构：从终点回溯到起点，转换为世界坐标
  void reconstructPath(Node* end_node, std::vector<PoseStamped>& path)
  {
    std::vector<Node*> temp_nodes;
    Node* current = end_node;

    // 回溯父节点
    while (current != nullptr) {
      temp_nodes.push_back(current);
      current = current->parent;
    }

    // 反转路径（起点→终点）
    std::reverse(temp_nodes.begin(), temp_nodes.end());

    // 转换为世界坐标 + PoseStamped 格式
    path.clear();
    for (auto& node : temp_nodes) {
      PoseStamped pose;
      costmap_->mapToWorld(static_cast<unsigned int>(node->x), static_cast<unsigned int>(node->y),
                          pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;
      // 方向默认朝东（简化，后续可优化）
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
      pose.header.frame_id = global_frame_;
      pose.header.stamp = clock_->now();
      path.push_back(pose);
    }

    RCLCPP_INFO(logger_, "重构路径点数：%zu", path.size());
  }

  // 核心成员变量（只保留必要项）
  nav2_costmap_2d::Costmap2D* costmap_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_, global_frame_;
  bool allow_unknown_;
  double map_resolution_;
};

}  // namespace custom_astar_planner

// 注册为 Nav2 插件
PLUGINLIB_EXPORT_CLASS(custom_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)