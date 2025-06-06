#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/QR>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using namespace std::chrono_literals;

//class PuzzlebotAsmc : public rclcpp::Node{
class PuzzlebotAsmc : public rclcpp_lifecycle::LifecycleNode{ 
  public:
    //PuzzlebotAsmc(): Node("puzzlebot_asmc_node"){
    PuzzlebotAsmc(): rclcpp_lifecycle::LifecycleNode("puzzlebot_asmc_node"){ 

      // Initialize variables 
      e << 0.0, 0.0;
      e_prev << 0.0, 0.0;
      e_dot << 0.0, 0.0;

      sigma << 0.0, 0.0;
      sigma_prev << 0.0, 0.0;
      
      r = 0.1; // Wheel radius
      //l = 0.5; // Wheel distance
      l = 0.168; // Wheel distance
      d = 0.1; // COM location
      kp = 0.15; // Proportional gain
      //kd = 0.5; // Derivative gain
      kd = 0.0; // Derivative gain

      u << 0.0, 0.0;
      x << 0.0, 0.0, 0.0;
      x_d << 0.0, 0.0, 0.0;
      x_d_dot << 0.0, 0.0, 0.0;

      uaux << 0.0, 0.0;
    
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Configuring Puzzlebot ASMC Node");

      estimator_pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&PuzzlebotAsmc::estimator_pose_callback, this, std::placeholders::_1));
      desired_pose_subscriber   = this->create_subscription<geometry_msgs::msg::PoseStamped>("/desired_pose", 10, std::bind(&PuzzlebotAsmc::desired_pose_callback, this, std::placeholders::_1));
      wheel_vel_publisher       = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      
      tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Activating Puzzlebot ASMC Node");
      
      // Start the timer
      // Activate publisher
      wheel_vel_publisher->on_activate();
      control_timer             = this->create_wall_timer(100ms, std::bind(&PuzzlebotAsmc::control_callback, this));

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Deactivating Puzzlebot ASMC Node");
      
      // Stop the timer
      // Deactivate publisher
      wheel_vel_publisher->on_deactivate();
      control_timer->cancel();
      
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Cleaning up Puzzlebot ASMC Node");
      
      // Unsubscribe from topics
      estimator_pose_subscriber.reset();
      desired_pose_subscriber.reset();
      wheel_vel_publisher.reset();
      
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //void estimator_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    void estimator_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){ 
      // Convert quaternion to euler angles (planar robot, so only yaw)
      //tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      //x << msg->pose.pose.position.x,
      //	   msg->pose.pose.position.y,
      //	   yaw;

      // Apply transform to the odometry data
      x(0) = msg->pose.pose.position.x; 
      x(1) = msg->pose.pose.position.y;
      x(2) = yaw; 


    }

    void desired_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    //void desired_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
      x_d << msg->pose.position.x, 
      	     msg->pose.position.y,
      	     0;
      //x_d << msg->point.x, 
      //	     msg->point.y,
      //	     0;
    }

    void control_callback(){
      // Define A matrix
      A << (r/2)*cos(x(2))+(d*r/(2*l))*sin(x(2)), (r/2)*cos(x(2))-(d*r/(2*l))*sin(x(2)),
           (r/2)*sin(x(2))-(d*r/(2*l))*cos(x(2)), (r/2)*sin(x(2))+(d*r/(2*l))*cos(x(2));

      // Compute error
      e << x_d(0) - x(0),
	   x_d(1) - x(1);
	   
      e_dot = (e - e_prev) / 0.01; 
      // If e_dot results in NaN because of zero division
      if (std::isnan(e_dot(0)) || std::isnan(e_dot(1))){
	e_dot << 0.0, 0.0;
      }  
				   
      e_prev = e;

      //std::cout << "e: " << e.transpose() << std::endl;
      //std::cout << "yaw: " << x(2) << std::endl;

      // Compute control input
      uaux = kp * e + kd * e_dot;

      sigma = A.colPivHouseholderQr().solve(uaux);

      // Saturate sigma
      sigma(0) = std::max(-0.05, std::min(0.05, sigma(0)));
      sigma(1) = std::max(-0.05, std::min(0.05, sigma(1)));

      // Publish sigma (wheel velocities)
      //cmd_vel.angular.x = sigma(0);
      //cmd_vel.angular.y = sigma(1);
      cmd_vel.linear.x  = 1.25*(sigma(0) + sigma(1)) / 2.0;
      cmd_vel.angular.z = 0.125*(sigma(1) - sigma(0)) / l;
      
      // Publish cmd_vel
      wheel_vel_publisher->publish(cmd_vel);
    }

  private:
    geometry_msgs::msg::Twist cmd_vel;

    Eigen::Matrix2d A;
    Eigen::Vector2d u;
    Eigen::Vector3d x_d;
    Eigen::Vector3d x_d_dot;
    Eigen::Vector2d uaux;
    Eigen::Vector3d x;
    Eigen::Vector2d e;
    Eigen::Vector2d e_prev;
    Eigen::Vector2d e_dot;
    Eigen::Vector2d sigma;
    Eigen::Vector2d sigma_prev;
    float r;
    float l;
    float d;
    float kp;
    float kd;

    rclcpp::TimerBase::SharedPtr control_timer; 
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_publisher;
    
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_subscriber;

    // Transform listener for map frame
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<PuzzlebotAsmc> puzzlebot_asmc_node = std::make_shared<PuzzlebotAsmc>();
  executor.add_node(puzzlebot_asmc_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  //rclcpp::spin(std::make_shared<PuzzlebotAsmc>());

  return 0;
}
