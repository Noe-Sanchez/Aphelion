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

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class GuildNavigator : public rclcpp::Node{
  public:
    GuildNavigator(): Node("guild_navigator_node"){
      this->declare_parameter<std::string>("map_file", "mapa.pgm");
      this->get_parameter("map_file", map_file);

      // Map subscriber
      map_subscriber      = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&GuildNavigator::map_callback, this, std::placeholders::_1));
      odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&GuildNavigator::odometry_callback, this, std::placeholders::_1));
      path_publisher      = this->create_publisher<visualization_msgs::msg::Marker>("/astar/path", 10);
      visited_publisher   = this->create_publisher<visualization_msgs::msg::MarkerArray>("/astar/visited", 10);
      goal_subscriber     = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&GuildNavigator::goal_callback, this, std::placeholders::_1));

      waypoint_timer = this->create_wall_timer(100ms, std::bind(&GuildNavigator::waypoint_callback, this)); 
      
      std::cout << "Init A* node" << std::flush;
      
      // Init empty map
      map_matrix = Eigen::MatrixXi::Zero(50, 50);

    }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    current_goal = *msg;
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


  void waypoint_callback(){}

  void astar(){
    std::cout << "Starting A* algorithm" << std::endl;
    // A* algorithm implementation

    // Init open and closed sets 
    closed_set.clear();
    open_set.clear();
   
    // Convert current odometry position to map coordinates
    int start_x = static_cast<int>((-current_map.info.origin.position.x + current_odometry.pose.pose.position.x) / current_map.info.resolution);
    int start_y = static_cast<int>((-current_map.info.origin.position.y + current_odometry.pose.pose.position.y) / current_map.info.resolution);
    int goal_x  = static_cast<int>((-current_map.info.origin.position.x + current_goal.pose.position.x) / current_map.info.resolution);
    int goal_y  = static_cast<int>((-current_map.info.origin.position.y + current_goal.pose.position.y) / current_map.info.resolution);

    // Check if start and goal are within map bounds
    if (start_x < 0 || start_x >= map_matrix.cols() || start_y < 0 || start_y >= map_matrix.rows() ||
      goal_x < 0 || goal_x >= map_matrix.cols() || goal_y < 0 || goal_y >= map_matrix.rows()) {
      RCLCPP_ERROR(this->get_logger(), "Start or goal position out of map bounds: start(%d, %d), goal(%d, %d)", start_x, start_y, goal_x, goal_y);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start position: (%d, %d), Goal position: (%d, %d)", start_x, start_y, goal_x, goal_y);

    // Check if start and goal are valid (not occupied, can be unknown or free)
    //if (map_matrix(start_y, start_x) != 0 || map_matrix(goal_y, goal_x) != 0) {
    if (map_matrix(start_y, start_x) == 100 || map_matrix(goal_y, goal_x) == 100) {
      RCLCPP_ERROR(this->get_logger(), "Start or goal position is occupied");
      return;
    }

    // Add start position to open set
    open_set.push_back(Eigen::Vector2i(start_x, start_y));
    auto current_node = open_set.back();

    current_path.points.clear();

    while (!open_set.empty()) {
      // Find the node with the lowest f score on the open set
      auto it = std::min_element(open_set.begin(), open_set.end(), [&](const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
				int f_a = (a - Eigen::Vector2i(goal_x, goal_y)).norm();
				int f_b = (b - Eigen::Vector2i(goal_x, goal_y)).norm();
				return f_a < f_b;
			}); 
      current_node = *it;
			open_set.erase(it);
			closed_set.push_back(current_node);

			// Check if we reached the goal
			if (current_node.x() == goal_x && current_node.y() == goal_y) {
				RCLCPP_INFO(this->get_logger(), "Reached goal at (%d, %d)", goal_x, goal_y);
				break;
			}

			// Check neighbors
			for (int dx = -1; dx <= 1; ++dx) {
				for (int dy = -1; dy <= 1; ++dy) {
					if ((dx == 0 && dy == 0) || (abs(dx) + abs(dy) != 1)) continue; // Skip self and diagonal moves

					int neighbor_x = current_node.x() + dx;
					int neighbor_y = current_node.y() + dy;

					// Check if neighbor is within bounds and not occupied
					//if (neighbor_x < 0 || neighbor_x >= map_matrix.cols() || neighbor_y < 0 || neighbor_y >= map_matrix.rows() ||
					//		map_matrix(neighbor_y, neighbor_x) !=0) {
					if (neighbor_x < 0 || neighbor_x >= map_matrix.cols() || neighbor_y < 0 || neighbor_y >= map_matrix.rows() ||
							map_matrix(neighbor_y, neighbor_x) == 100) {
						continue;
					}

					Eigen::Vector2i neighbor(neighbor_x, neighbor_y);

					// If the neighbor is already in closed set, skip it
					if (std::find(closed_set.begin(), closed_set.end(), neighbor) != closed_set.end()) {
						continue;
					}

					// If the neighbor is not in open set, add it
					if (std::find(open_set.begin(), open_set.end(), neighbor) == open_set.end()) {
						open_set.push_back(neighbor);
					}
				}
			}


    }
   
    // If we reached here, we either found the goal or exhausted all possibilities
    if (open_set.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No path found to the goal (%d, %d)", goal_x, goal_y);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Pathfinding completed, preparing to visualize path");

    // For now, direct pose line to goal
    current_path.header.frame_id = "map";
    current_path.header.stamp = this->now();
    current_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    current_path.action = visualization_msgs::msg::Marker::ADD;
    current_path.scale.x = 0.1;
    current_path.color.r = 0.0f;
    current_path.color.g = 1.0f;
    current_path.color.b = 0.0f;
    current_path.color.a = 1.0f;

    geometry_msgs::msg::Point start_point;
    start_point.x = current_odometry.pose.pose.position.x;
    start_point.y = current_odometry.pose.pose.position.y;
    start_point.z = 0.0;
    current_path.points.push_back(start_point);
    for (const auto& node : closed_set) {
			geometry_msgs::msg::Point point;
			point.x = node.x() * current_map.info.resolution + current_map.info.origin.position.x;
			point.y = node.y() * current_map.info.resolution + current_map.info.origin.position.y;
			point.z = 0.0;
			current_path.points.push_back(point);
		}
    geometry_msgs::msg::Point goal_point;
    goal_point.x = current_goal.pose.position.x;
    goal_point.y = current_goal.pose.position.y;
    goal_point.z = 0.0;
    current_path.points.push_back(goal_point);

    path_publisher->publish(current_path);
    
    // Visualize visited nodes
    visited_nodes.markers.clear();
    visualization_msgs::msg::Marker visited_marker;
    visited_marker.header.frame_id = "map";
    visited_marker.header.stamp = this->now();
    visited_marker.type = visualization_msgs::msg::Marker::CUBE;
    visited_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visited_marker.scale.x = current_map.info.resolution;
    visited_marker.scale.y = current_map.info.resolution;
    visited_marker.scale.z = current_map.info.resolution;
    visited_marker.color.r = 1.0f;
    visited_marker.color.g = 0.0f;
    visited_marker.color.b = 0.0f;
    visited_marker.color.a = 0.5f;
    visited_marker.id = 0;
    visited_nodes.markers.push_back(visited_marker);
    visited_publisher->publish(visited_nodes);
    visited_marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto& node : closed_set) {
      visited_marker.pose.position.x = node.x() * current_map.info.resolution + current_map.info.origin.position.x + current_map.info.resolution / 2.0;
      visited_marker.pose.position.y = node.y() * current_map.info.resolution + current_map.info.origin.position.y + current_map.info.resolution / 2.0;
      visited_marker.pose.position.z = 0.01; // Slightly above the ground
      visited_marker.id = node.x() + node.y() * 1000; // Unique ID for each marker
      visited_nodes.markers.push_back(visited_marker);
    }
    visited_publisher->publish(visited_nodes);
  }	

  private:
    std::string map_file;

    Eigen::MatrixXi map_matrix;
    std::vector<Eigen::Vector2i> closed_set;
    std::vector<Eigen::Vector2i> open_set;

    nav_msgs::msg::OccupancyGrid current_map;
    nav_msgs::msg::Odometry current_odometry;
    visualization_msgs::msg::Marker current_path;
    visualization_msgs::msg::MarkerArray visited_nodes;
    geometry_msgs::msg::PoseStamped current_goal;
    geometry_msgs::msg::Pose old_origin;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visited_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber;

    rclcpp::TimerBase::SharedPtr waypoint_timer;


};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuildNavigator>());
  rclcpp::shutdown();
  return 0;
}
