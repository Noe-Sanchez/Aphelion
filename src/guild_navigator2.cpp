#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include <fstream>
#include <sstream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

// Node structure for A* algorithm
struct AStarNode {
    int x, y;
    double g_score;  // Cost from start
    double f_score;  // g_score + heuristic
    int parent_x, parent_y;
    
    AStarNode(int x = 0, int y = 0, double g = 0.0, double f = 0.0, int px = -1, int py = -1)
        : x(x), y(y), g_score(g), f_score(f), parent_x(px), parent_y(py) {}
    
    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

// Hash function for std::pair<int, int> to use in unordered_map/set
struct PairHash {
    size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

class GuildNavigator : public rclcpp::Node{
  public:
    GuildNavigator(): Node("guild_navigator_node"){
      this->declare_parameter<std::string>("map_file", "mapa.pgm");
      this->get_parameter("map_file", map_file);

      // Map subscriber
      map_subscriber      = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&GuildNavigator::map_callback, this, std::placeholders::_1));
      odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&GuildNavigator::odometry_callback, this, std::placeholders::_1));
      path_publisher      = this->create_publisher<visualization_msgs::msg::Marker>("/astar/path", 10);
      visited_publisher   = this->create_publisher<visualization_msgs::msg::MarkerArray>("/astar/visited", 10);
      goal_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&GuildNavigator::goal_callback, this, std::placeholders::_1));

      target_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/desired_pose", 10);
      
      std::cout << "Init A* node" << std::flush;
      
      // Init empty map
      map_matrix = Eigen::MatrixXi::Zero(50, 50);
    }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    current_goal = *msg;
    astar(); // Call A* algorithm when goal is received
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    current_odometry = *msg;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    std::cout << "Map received" << std::endl;
    current_map = *msg;

    // Update map matrix from occupancy grid
    int width = msg->info.width;
    int height = msg->info.height;
    map_matrix.resize(height, width);
    
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        map_matrix(i, j) = msg->data[i * width + j];
      }
    }

    astar(); // Call A* algorithm when map is received
  }

  // Heuristic function (Euclidean distance)
  double heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  // Check if a position is valid (within bounds and not occupied)
  bool isValid(int x, int y) {
    if (x < 0 || x >= map_matrix.cols() || y < 0 || y >= map_matrix.rows()) {
      return false;
    }
    return map_matrix(y, x) != 100; // Not occupied
  }

  // Check if position is too close to walls (safety margin)
  bool isSafe(int x, int y, int safety_radius = 3) {
    for (int dx = -safety_radius; dx <= safety_radius; ++dx) {
      for (int dy = -safety_radius; dy <= safety_radius; ++dy) {
        int check_x = x + dx;
        int check_y = y + dy;
        if (check_x >= 0 && check_x < map_matrix.cols() && 
            check_y >= 0 && check_y < map_matrix.rows()) {
          if (map_matrix(check_y, check_x) == 100) {
            return false;
          }
        }
      }
    }
    return true;
  }

  // Reconstruct path from goal to start using parent pointers
  std::vector<std::pair<int, int>> reconstructPath(
      const std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash>& parent_map,
      std::pair<int, int> current) {
    
    std::vector<std::pair<int, int>> path;
    while (parent_map.find(current) != parent_map.end()) {
      path.push_back(current);
      current = parent_map.at(current);
    }
    path.push_back(current); // Add start position
    std::reverse(path.begin(), path.end());
    return path;
  }

  void waypoint_callback(){
    if (optimal_path.empty()) return;

    // Get current waypoint from optimal path
    if (current_waypoint >= optimal_path.size()) {
      RCLCPP_INFO(this->get_logger(), "Reached the last waypoint, stopping navigation");
      waypoint_timer->cancel();
      current_waypoint = 0;
      return;
    }

    auto& waypoint = optimal_path[current_waypoint];
    
    // Convert grid coordinates back to world coordinates
    current_target.pose.position.x = waypoint.first * current_map.info.resolution + 
                                     current_map.info.origin.position.x + 
                                     current_map.info.resolution / 2.0;
    current_target.pose.position.y = waypoint.second * current_map.info.resolution + 
                                     current_map.info.origin.position.y + 
                                     current_map.info.resolution / 2.0;
    current_target.pose.position.z = 0.0;
    current_target.header.frame_id = "map";
    current_target.header.stamp = this->now();
    target_publisher->publish(current_target);

    // Compute distance to current waypoint
    double norm2 = std::sqrt(
      std::pow(current_odometry.pose.pose.position.x - current_target.pose.position.x, 2) +
      std::pow(current_odometry.pose.pose.position.y - current_target.pose.position.y, 2)
    );

    if (norm2 < 0.075) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %d at (%f, %f)", 
                  current_waypoint, current_target.pose.position.x, current_target.pose.position.y);
      current_waypoint++;
    } else {
      RCLCPP_INFO(this->get_logger(), "Moving to waypoint %d at (%f, %f), distance: %f", 
                  current_waypoint, current_target.pose.position.x, current_target.pose.position.y, norm2);
    }
  }

  void astar(){
    auto initial_time = this->now();
    std::cout << "Starting A* algorithm" << std::endl;

    // Convert current odometry position to map coordinates
    int start_x = static_cast<int>((-current_map.info.origin.position.x + current_odometry.pose.pose.position.x) / current_map.info.resolution);
    int start_y = static_cast<int>((-current_map.info.origin.position.y + current_odometry.pose.pose.position.y) / current_map.info.resolution);
    int goal_x  = static_cast<int>((-current_map.info.origin.position.x + current_goal.pose.position.x) / current_map.info.resolution);
    int goal_y  = static_cast<int>((-current_map.info.origin.position.y + current_goal.pose.position.y) / current_map.info.resolution);

    // Check if start and goal are within map bounds
    if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
      RCLCPP_ERROR(this->get_logger(), "Start or goal position out of bounds or occupied");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start: (%d, %d), Goal: (%d, %d)", start_x, start_y, goal_x, goal_y);

    // A* data structures
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_set<std::pair<int, int>, PairHash> open_set_hash;
    std::unordered_set<std::pair<int, int>, PairHash> closed_set;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> parent_map;
    std::unordered_map<std::pair<int, int>, double, PairHash> g_score;

    // Initialize start node
    std::pair<int, int> start_pos(start_x, start_y);
    std::pair<int, int> goal_pos(goal_x, goal_y);
    
    g_score[start_pos] = 0.0;
    double h_start = heuristic(start_x, start_y, goal_x, goal_y);
    open_set.push(AStarNode(start_x, start_y, 0.0, h_start));
    open_set_hash.insert(start_pos);

    // 8-directional movement (including diagonals)
    std::vector<std::pair<int, int>> directions = {
      {-1, -1}, {-1,  0}, {-1,  1},
      { 0, -1},           { 0,  1},
      { 1, -1}, { 1,  0}, { 1,  1}
    };

    std::vector<std::pair<int, int>> visited_nodes_list;
    bool path_found = false;

    while (!open_set.empty()) {
      // Get node with lowest f_score
      AStarNode current = open_set.top();
      open_set.pop();
      
      std::pair<int, int> current_pos(current.x, current.y);
      open_set_hash.erase(current_pos);
      closed_set.insert(current_pos);
      visited_nodes_list.push_back(current_pos);

      // Check if we reached the goal
      if (current.x == goal_x && current.y == goal_y) {
        RCLCPP_INFO(this->get_logger(), "Path found!");
        path_found = true;
        break;
      }

      // Explore neighbors
      for (const auto& dir : directions) {
        int neighbor_x = current.x + dir.first;
        int neighbor_y = current.y + dir.second;
        std::pair<int, int> neighbor_pos(neighbor_x, neighbor_y);

        // Skip if invalid, unsafe, or already in closed set
        if (!isValid(neighbor_x, neighbor_y) || 
            !isSafe(neighbor_x, neighbor_y) ||
            closed_set.find(neighbor_pos) != closed_set.end()) {
          continue;
        }

        // Calculate movement cost (higher for diagonal moves)
        double move_cost = (abs(dir.first) + abs(dir.second) == 2) ? 1.414 : 1.0;
        double tentative_g = g_score[current_pos] + move_cost;

        // If this path to neighbor is better than any previous one
        if (g_score.find(neighbor_pos) == g_score.end() || tentative_g < g_score[neighbor_pos]) {
          parent_map[neighbor_pos] = current_pos;
          g_score[neighbor_pos] = tentative_g;
          double h_score = heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
          double f_score = tentative_g + h_score;

          if (open_set_hash.find(neighbor_pos) == open_set_hash.end()) {
            open_set.push(AStarNode(neighbor_x, neighbor_y, tentative_g, f_score));
            open_set_hash.insert(neighbor_pos);
          }
        }
      }
    }

    if (!path_found) {
      RCLCPP_ERROR(this->get_logger(), "No path found to goal");
      return;
    }

    // Reconstruct optimal path
    optimal_path = reconstructPath(parent_map, goal_pos);
    
    RCLCPP_INFO(this->get_logger(), "Path length: %ld nodes", optimal_path.size());

    // Visualize the optimal path
    current_path.header.frame_id = "map";
    current_path.header.stamp = this->now();
    current_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    current_path.action = visualization_msgs::msg::Marker::ADD;
    current_path.scale.x = 0.05;
    current_path.color.r = 0.0f;
    current_path.color.g = 1.0f;
    current_path.color.b = 0.0f;
    current_path.color.a = 1.0f;
    current_path.points.clear();

    for (const auto& node : optimal_path) {
      geometry_msgs::msg::Point point;
      point.x = node.first * current_map.info.resolution + current_map.info.origin.position.x + current_map.info.resolution / 2.0;
      point.y = node.second * current_map.info.resolution + current_map.info.origin.position.y + current_map.info.resolution / 2.0;
      point.z = 0.1;
      current_path.points.push_back(point);
    }

    path_publisher->publish(current_path);
    
    // Visualize visited nodes (for debugging)
    visited_nodes.markers.clear();
    visited_marker.header.frame_id = "map";
    visited_marker.header.stamp = this->now();
    visited_marker.type = visualization_msgs::msg::Marker::CUBE;
    visited_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visited_marker.scale.x = current_map.info.resolution * 0.5;
    visited_marker.scale.y = current_map.info.resolution * 0.5;
    visited_marker.scale.z = current_map.info.resolution * 0.5;
    visited_marker.color.r = 1.0f;
    visited_marker.color.g = 0.0f;
    visited_marker.color.b = 0.0f;
    visited_marker.color.a = 0.3f;
    visited_marker.id = 0;
    visited_nodes.markers.push_back(visited_marker);
    visited_publisher->publish(visited_nodes);
    
    visited_nodes.markers.clear();
    visited_marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto& node : visited_nodes_list) {
      visited_marker.pose.position.x = node.first * current_map.info.resolution + current_map.info.origin.position.x + current_map.info.resolution / 2.0;
      visited_marker.pose.position.y = node.second * current_map.info.resolution + current_map.info.origin.position.y + current_map.info.resolution / 2.0;
      visited_marker.pose.position.z = 0.01;
      visited_marker.id = node.first + node.second * 1000;
      visited_nodes.markers.push_back(visited_marker);
    }
    visited_publisher->publish(visited_nodes);

    auto elapsed_time = this->now() - initial_time;
    RCLCPP_INFO(this->get_logger(), "A* completed in %ld ms", elapsed_time.nanoseconds() / 1000000);

    // Start waypoint following
    current_waypoint = 0;
    if (!waypoint_timer) {
      waypoint_timer = this->create_wall_timer(100ms, std::bind(&GuildNavigator::waypoint_callback, this));
    } else {
      waypoint_timer->reset();
    }
  }	

  private:
    std::string map_file;
    int current_waypoint = 0;

    Eigen::MatrixXi map_matrix;
    std::vector<std::pair<int, int>> optimal_path;

    nav_msgs::msg::OccupancyGrid current_map;
    nav_msgs::msg::Odometry current_odometry;
    visualization_msgs::msg::Marker current_path;
    visualization_msgs::msg::MarkerArray visited_nodes;
    geometry_msgs::msg::PoseStamped current_goal;
    visualization_msgs::msg::Marker visited_marker;
    geometry_msgs::msg::PoseStamped current_target;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visited_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher;

    rclcpp::TimerBase::SharedPtr waypoint_timer;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuildNavigator>());
  rclcpp::shutdown();
  return 0;
}
