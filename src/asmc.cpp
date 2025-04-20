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
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/QR>

using namespace std::chrono_literals;

class PuzzlebotAsmc : public rclcpp::Node{
  public:
    PuzzlebotAsmc(): Node("puzzlebot_asmc_node"){
      // Get puzzlebot id from launchfile parameter
      this->declare_parameter("puzzlebot_id", 0);
      this->get_parameter("puzzlebot_id", puzzlebot_id);

      this->declare_parameter("use_prefix", false);
      this->get_parameter("use_prefix", use_prefix);

      // Subscribers
      //estimator_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/estimator/pose/", 10, std::bind(&PuzzlebotAsmc::estimator_pose_callback, this, std::placeholders::_1));
      //desired_pose_subscriber   = this->create_subscription<geometry_msgs::msg::PoseStamped>("/desired_pose/", 10, std::bind(&PuzzlebotAsmc::desired_pose_callback, this, std::placeholders::_1));
      if (use_prefix){
	estimator_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/estimator/pose", 10, std::bind(&PuzzlebotAsmc::estimator_pose_callback, this, std::placeholders::_1));
	desired_pose_subscriber   = this->create_subscription<geometry_msgs::msg::PoseStamped>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/desired_pose", 10, std::bind(&PuzzlebotAsmc::desired_pose_callback, this, std::placeholders::_1));
	wheel_vel_publisher       = this->create_publisher<geometry_msgs::msg::Twist>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/cmd_vel", 10);
      } else {
	estimator_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/estimator/pose", 10, std::bind(&PuzzlebotAsmc::estimator_pose_callback, this, std::placeholders::_1));
	desired_pose_subscriber   = this->create_subscription<geometry_msgs::msg::PoseStamped>("/desired_pose", 10, std::bind(&PuzzlebotAsmc::desired_pose_callback, this, std::placeholders::_1));
	wheel_vel_publisher       = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      }
      // Make 0.5s timer
      control_timer             = this->create_wall_timer(10ms, std::bind(&PuzzlebotAsmc::control_callback, this));

      // Initialize variables 
      e << 0.0, 0.0;
      e_prev << 0.0, 0.0;
      e_dot << 0.0, 0.0;

      sigma << 0.0, 0.0;
      sigma_prev << 0.0, 0.0;
      
      r = 0.1; // Wheel radius
      l = 0.5; // Wheel distance
      d = 0.1; // COM location
      kp = 0.5; // Proportional gain
      kd = 0.5; // Derivative gain

      u << 0.0, 0.0;
      x << 0.0, 0.0, 0.0;
      x_d << 0.0, 0.0, 0.0;
      x_d_dot << 0.0, 0.0, 0.0;

      uaux << 0.0, 0.0;

    }

    void estimator_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      // Convert quaternion to euler angles (planar robot, so only yaw)
      tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      x << msg->pose.position.x, 
	   msg->pose.position.y,
	   yaw;

    }

    void desired_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      x_d << msg->pose.position.x, 
	     msg->pose.position.y,
	     0;
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

      std::cout << "e: " << e.transpose() << std::endl;
      std::cout << "yaw: " << x(2) << std::endl;

      // Compute control input
      uaux = kp * e + kd * e_dot;

      sigma = A.colPivHouseholderQr().solve(uaux);

      // Saturate sigma
      //sigma(0) = std::max(-3.0, std::min(3.0, sigma(0)));
      //sigma(1) = std::max(-3.0, std::min(3.0, sigma(1)));
      sigma(0) = std::max(-2.0, std::min(2.0, sigma(0)));
      sigma(1) = std::max(-2.0, std::min(2.0, sigma(1)));

      // Publish sigma (wheel velocities)
      cmd_vel.angular.x = sigma(0);
      cmd_vel.angular.y = sigma(1);
      
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
    bool use_prefix;

    rclcpp::TimerBase::SharedPtr control_timer; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_publisher;

    int puzzlebot_id;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PuzzlebotAsmc>());
  rclcpp::shutdown();
  return 0;
}
