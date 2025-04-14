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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class SimpleDynamics : public rclcpp::Node{
  public:
    SimpleDynamics(): Node("puzzlebot_kinematics_node"){
      // Get puzzlebot id from launchfile parameter
      this->declare_parameter("puzzlebot_id", 0);
      this->get_parameter("puzzlebot_id", puzzlebot_id);
      this->declare_parameter("use_prefix", false);
      this->get_parameter("use_prefix", use_prefix);

      // Subscribers
      //reset_state_subscriber       = this->create_subscription<std_msgs::msg::Bool>("/reset_state", 10,  std::bind(&SimpleDynamics::reset_state_callback, this, std::placeholders::_1));
      //control_input_subscriber     = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&SimpleDynamics::control_input_callback, this, std::placeholders::_1));
      if (use_prefix){
	reset_state_subscriber       = this->create_subscription<std_msgs::msg::Bool>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/reset_state", 10,  std::bind(&SimpleDynamics::reset_state_callback, this, std::placeholders::_1));
	control_input_subscriber     = this->create_subscription<geometry_msgs::msg::Twist>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/cmd_vel", 10, std::bind(&SimpleDynamics::control_input_callback, this, std::placeholders::_1));
	estimator_pose_publisher     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/estimator/pose", 10);
	estimator_velocity_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/estimator/velocity", 10);
      } else {
	reset_state_subscriber       = this->create_subscription<std_msgs::msg::Bool>("/reset_state", 10,  std::bind(&SimpleDynamics::reset_state_callback, this, std::placeholders::_1));
	control_input_subscriber     = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&SimpleDynamics::control_input_callback, this, std::placeholders::_1));
	estimator_pose_publisher     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimator/pose", 10);
	estimator_velocity_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/estimator/velocity", 10);
      }

      transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      // Make 0.5s timer
      dynamics_timer = this->create_wall_timer(10ms, std::bind(&SimpleDynamics::dynamics_callback, this));
  

      // Initialize variables 
      p << 0.0, 0.0, 0.0;
      p_dot << 0.0, 0.0, 0.0;
      p_dot_prev << 0.0, 0.0, 0.0;

      r = 0.1; // Wheel radius
      l = 0.5; // Wheel distance

    }

    void reset_state_callback(const std_msgs::msg::Bool::SharedPtr msg){
      if (msg->data){
	p << 0.0, 0.0, 0.0;
	p_dot << 0.0, 0.0, 0.0;
	p_dot_prev << 0.0, 0.0, 0.0;
	w << 0.0, 0.0;
      }
    }

    void control_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      w << msg->angular.x, 
	   msg->angular.y;   
    }

    void dynamics_callback(){
      p_dot << r*(w[1] + w[0])*0.5 * cos(p[2]),
	       r*(w[1] + w[0])*0.5 * sin(p[2]),
	       r*(w[1] - w[0])/l;
      // Trapezoidal integration 
      p << p + (p_dot + p_dot_prev) * 0.5 * 0.01;

      p_dot_prev << p_dot[0], p_dot[1], p_dot[2];

      std::cout << "p: " << p.transpose() << std::endl;
      std::cout << "p_dot: " << p_dot.transpose() << std::endl;

      // Transform theta to new angle between -pi and pi
      p[2] = fmod(p[2] + M_PI, 2*M_PI) - M_PI;

      std::cout << "p: " << p.transpose() << std::endl;
      std::cout << "p_dot: " << p_dot.transpose() << std::endl;

      // Update pose
      pose.pose.position.x = p[0];
      pose.pose.position.y = p[1];
      pose.pose.position.z = 0.0;
      // Temporary variable to store quaternion
      Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(p[2], Eigen::Vector3d::UnitZ()));
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      pose.header.frame_id = "world";
      pose.header.stamp = this->now();

      transform.header.stamp = this->now();
      transform.header.frame_id = "world";
      //transform.child_frame_id = "base_footprint";
      if (use_prefix){
	transform.child_frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_base_footprint";
      } else {
	transform.child_frame_id = "base_footprint";
      }
      transform.transform.translation.x = p[0];
      transform.transform.translation.y = p[1];
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.x = q.x();
      transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z();
      transform.transform.rotation.w = q.w();

      transform_broadcaster->sendTransform(transform);
      estimator_pose_publisher->publish(pose);
    }

  private:
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::TwistStamped pose_dot;
    geometry_msgs::msg::TransformStamped transform;

    Eigen::Vector3d p;
    Eigen::Vector3d p_dot;
    Eigen::Vector3d p_dot_prev;
    Eigen::Vector2d w; // Wheel left and right velocities
    float r;
    float l;
    bool use_prefix;

    rclcpp::TimerBase::SharedPtr dynamics_timer; 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_input_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr estimator_velocity_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;

    int puzzlebot_id;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleDynamics>());
  rclcpp::shutdown();
  return 0;
}
