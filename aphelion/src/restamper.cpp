#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

class Restamper : public rclcpp::Node{
  public:
    Restamper(): Node("restamper_node"){
      // Declare argument bool
      this->declare_parameter<bool>("use_gz_odom", false);
      this->get_parameter("use_gz_odom", use_gz_odom);
      RCLCPP_INFO(this->get_logger(), "use_gz_odom: %s", use_gz_odom ? "true" : "false");

      nanosecond_pub = this->create_publisher<std_msgs::msg::UInt32>("/sim_time", 10);

      lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Restamper::lidar_callback, this, std::placeholders::_1));
      lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan2", 10);

      odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Restamper::odom_callback, this, std::placeholders::_1));
      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
      tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      wheel_l_pub = this->create_publisher<std_msgs::msg::Float32>("/VelocityEncL", 10);
      wheel_r_pub = this->create_publisher<std_msgs::msg::Float32>("/VelocityEncR", 10);

      encoder_timer = this->create_wall_timer(50ms, std::bind(&Restamper::encoder_timer_callback, this));

      trans << 20.0,  1.68,
	       20.0, -1.68;
      vels       << 0.0, 0.0;
      wheel_vels << 0.0, 0.0;
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
      
      sim_time_msg.data = msg->header.stamp.nanosec;
      nanosecond_pub->publish(sim_time_msg);

      odom_msg = *msg;
      odom_msg.header.stamp = this->now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_footprint";

      //transform_msg.transform.translation.x = -msg->pose.pose.position.y + 0.284;
      //transform_msg.transform.translation.y = msg->pose.pose.position.x + 0.296;
      transform_msg.transform.translation.x = msg->pose.pose.position.x; 
      transform_msg.transform.translation.y = msg->pose.pose.position.y;
      transform_msg.transform.translation.z = msg->pose.pose.position.z;

      transform_msg.transform.rotation = msg->pose.pose.orientation;
      
      //transform_msg.transform.rotation.w = msg->pose.pose.orientation.w * 0.7071 - msg->pose.pose.orientation.z * 0.7071;
      //transform_msg.transform.rotation.z = msg->pose.pose.orientation.w * 0.7071 + msg->pose.pose.orientation.z * 0.7071;
      
      odom_msg.pose.pose.position.x = transform_msg.transform.translation.x;
      odom_msg.pose.pose.position.y = transform_msg.transform.translation.y;

      odom_msg.pose.pose.orientation = transform_msg.transform.rotation;
      
      if (use_gz_odom){
	tf_broadcaster->sendTransform(transform_msg);
	odom_pub->publish(odom_msg);
      }

      // Publish wheel velocities
      std_msgs::msg::Float32 wheel_msg;

      vels(0) = msg->twist.twist.linear.x;
      vels(1) = msg->twist.twist.angular.z;

      wheel_vels = trans * vels;
      wheel_l_msg.data = wheel_vels(1);
      wheel_r_msg.data = wheel_vels(0);

    }

    void encoder_timer_callback(){
      wheel_l_pub->publish(wheel_l_msg);
      wheel_r_pub->publish(wheel_r_msg);
    }

  private:
    bool use_gz_odom;
    sensor_msgs::msg::LaserScan lidar_msg;
    nav_msgs::msg::Odometry odom_msg;
    geometry_msgs::msg::TransformStamped transform_msg;
    std_msgs::msg::Float32 wheel_l_msg;
    std_msgs::msg::Float32 wheel_r_msg;
    std_msgs::msg::UInt32 sim_time_msg;
    Eigen::Matrix2d trans;
    Eigen::Vector2d vels;
    Eigen::Vector2d wheel_vels;

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr nanosecond_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub; 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_l_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheel_r_pub;
    rclcpp::TimerBase::SharedPtr encoder_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Restamper>());
  rclcpp::shutdown();
  return 0;
}
