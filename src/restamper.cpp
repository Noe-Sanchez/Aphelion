#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class Restamper : public rclcpp::Node{
  public:
    Restamper(): Node("restamper_node"){
      lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Restamper::lidar_callback, this, std::placeholders::_1));
      lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan2", 10);

      odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Restamper::odom_callback, this, std::placeholders::_1));
      tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
      lidar_msg = *msg;
      lidar_msg.header.stamp = this->now();
      lidar_msg.header.frame_id = "laser_link";
      lidar_pub->publish(lidar_msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      transform_msg.header.stamp = this->now();
      transform_msg.header.frame_id = "odom";
      transform_msg.child_frame_id = "base_footprint";
      
      transform_msg.transform.translation.x = msg->pose.pose.position.x;
      transform_msg.transform.translation.y = msg->pose.pose.position.y;
      transform_msg.transform.translation.z = msg->pose.pose.position.z;
      
      transform_msg.transform.rotation = msg->pose.pose.orientation;
      
      tf_broadcaster->sendTransform(transform_msg);
    }

  private:
    sensor_msgs::msg::LaserScan lidar_msg;
    geometry_msgs::msg::TransformStamped transform_msg;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    // Transform publisher
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Restamper>());
  rclcpp::shutdown();
  return 0;
}
