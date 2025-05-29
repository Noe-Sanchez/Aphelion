#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class GuildNavigator : public rclcpp::Node{
  public:
    GuildNavigator(): Node("guild_navigator_node"){
      // Map subscriber
      map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&GuildNavigator::map_callback, this, std::placeholders::_1));
      odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&GuildNavigator::odometry_callback, this, std::placeholders::_1));
      path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("astar/path", 10);
      goal_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 10, std::bind(&GuildNavigator::goal_callback, this, std::placeholders::_1)); 
      
      // Init map (all free)
      map_matrix = Eigen::MatrixXi::Zero(50, 50); 
    }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    current_goal = *msg;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    current_odometry = *msg;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    // Convert the occupancy grid to an Eigen matrix
    // Copy existing map onto new map using origin 
    

    int width = msg->info.width;
    int height = msg->info.height;
    geometry_msgs::msg::Pose origin = msg->info.origin;
    map_matrix.resize(height, width);
    
    for (int i = 0; i < height; ++i) {
    	for (int j = 0; j < width; ++j) {
    		map_matrix(i, j) = msg->data[i * width + j];
    	}
    }

    // Static hardcoded point in 25, 25 cell of the map
    current_path.header.frame_id = "map";
    current_path.header.stamp = this->now();
    current_path.type = visualization_msgs::msg::Marker::CUBE;
    current_path.action = visualization_msgs::msg::Marker::ADD;
    current_path.scale.x = 0.15;
    current_path.scale.y = 0.15;
    current_path.scale.z = 0.15;
    current_path.color.r = 1.0f;
    current_path.color.g = 0.0f;
    current_path.color.b = 0.0f;
    current_path.color.a = 1.0f;
    // Apply transformation of old origin as well
    current_path.pose.position.x = origin.position.x + 25 * msg->info.resolution + old_origin.position.x;
    current_path.pose.position.y = origin.position.y + 25 * msg->info.resolution + old_origin.position.y;
    current_path.pose.position.z = 0.0;
    
    path_publisher->publish(current_path);

    old_origin = origin; 

    /*
    // For now, direct pose line to goal
    current_path.points.clear();
    current_path.header.frame_id = "map";
    current_path.header.stamp = this->now();
    current_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    current_path.action = visualization_msgs::msg::Marker::ADD;
    current_path.scale.x = 0.1;
    current_path.color.r = 0.0f;
    current_path.color.g = 1.0f;
    current_path.color.b = 0.0f;
    current_path.color.a = 1.0f;


    current_path.points.push_back(current_odometry.pose.pose.position);
    for (int i = 0; i < 5; ++i) {
      geometry_msgs::msg::Point point;
      point.x = current_odometry.pose.pose.position.x + i * (current_goal.pose.position.x - current_odometry.pose.pose.position.x) / 5;
      point.y = current_odometry.pose.pose.position.y + i * (current_goal.pose.position.y - current_odometry.pose.pose.position.y) / 5;
      point.z = 0.0;
      current_path.points.push_back(point);
    }
    current_path.points.push_back(current_goal.pose.position);
    path_publisher->publish(current_path);*/
    
  }

  private:
    // Eigen matrix for storing the map
    Eigen::MatrixXi map_matrix;
    // Closed and open sets for A* algorithm
    std::vector<Eigen::Vector2i> closed_set;
    std::vector<Eigen::Vector2i> open_set;
    nav_msgs::msg::Odometry current_odometry;
    visualization_msgs::msg::Marker current_path;
    geometry_msgs::msg::PoseStamped current_goal;
    geometry_msgs::msg::Pose old_origin;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuildNavigator>());
  rclcpp::shutdown();
  return 0;
}
