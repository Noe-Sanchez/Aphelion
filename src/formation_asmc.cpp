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

class FormationAsmc : public rclcpp::Node{
  public:
    FormationAsmc(): Node("formation_asmc_node"){
      // Get puzzlebot id from launchfile parameter
      this->declare_parameter("robot_id", 0);
      this->get_parameter("robot_id", robot_id);

      this->declare_parameter("robot_name", "puzzlebot");
      this->get_parameter("robot_name", robot_name);

      // Subscribers
      estimator_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(robot_name + "_" + std::to_string(robot_id) + "/estimator/pose", 10, std::bind(&FormationAsmc::estimator_pose_callback, this, std::placeholders::_1));
      desired_pose_subscriber   = this->create_subscription<geometry_msgs::msg::PoseStamped>(robot_name + "_" + std::to_string(robot_id) + "/desired_pose", 10, std::bind(&FormationAsmc::desired_pose_callback, this, std::placeholders::_1));
      leader_pose_subscriber   = this->create_subscription<geometry_msgs::msg::PoseStamped>(robot_name + "_leader/pose", 10, std::bind(&FormationAsmc::leader_pose_callback, this, std::placeholders::_1));
      leader_velocity_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>(robot_name + "_leader/velocity", 10, std::bind(&FormationAsmc::leader_velocity_callback, this, std::placeholders::_1));

      // Publishers
      wheel_vel_publisher       = this->create_publisher<geometry_msgs::msg::Twist>(robot_name + "_" + std::to_string(robot_id) + "/cmd_vel", 10);
      world_desired_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(robot_name + "_" + std::to_string(robot_id) + "/world_desired_pose", 10);

      // Make 0.1s timer
      control_timer             = this->create_wall_timer(10ms, std::bind(&FormationAsmc::control_callback, this));

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
      x_d_l << 0.0, 0.0, 0.0;
      x_d_l_dot << 0.0, 0.0, 0.0;
      x_d_w << 0.0, 0.0, 0.0;
      x_d_w_dot << 0.0, 0.0, 0.0;

      leader_pose << 0.0, 0.0, 0.0;
      leader_velocity << 0.0, 0.0, 0.0;

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
      x_d_l << msg->pose.position.x, 
	       msg->pose.position.y,
	       0;
    }

    void leader_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      // Convert quaternion to euler angles (planar robot, so only yaw)
      tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      leader_pose << msg->pose.position.x, 
                     msg->pose.position.y,
                     yaw;
    }

    void leader_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
      leader_velocity << msg->twist.linear.x, 
                         msg->twist.linear.y,
                         msg->twist.angular.z;
    }

    void control_callback(){
      // Define A matrix
      A << (r/2)*cos(x(2))+(d*r/(2*l))*sin(x(2)), (r/2)*cos(x(2))-(d*r/(2*l))*sin(x(2)),
           (r/2)*sin(x(2))-(d*r/(2*l))*cos(x(2)), (r/2)*sin(x(2))+(d*r/(2*l))*cos(x(2));

      // Compute error
      x_d_w << leader_pose(0) + x_d_l(0)*cos(leader_pose(2)) - x_d_l(1)*sin(leader_pose(2)),
	       leader_pose(1) + x_d_l(0)*sin(leader_pose(2)) + x_d_l(1)*cos(leader_pose(2)),
	       0;

      std::cout << "Computed world pose x_d_w: " << x_d_w.transpose() << std::endl;
      
      e << x_d_w(0) - x(0),
	   x_d_w(1) - x(1);
	   
      e_dot = (e - e_prev) / 0.01; 
      // If e_dot results in NaN because of zero division
      if (std::isnan(e_dot(0)) || std::isnan(e_dot(1))){
	e_dot << 0.0, 0.0;
      }  
				   
      e_prev = e;


      // Compute control input
      uaux = kp * e + kd * e_dot;

      sigma = A.colPivHouseholderQr().solve(uaux);

      // Saturate sigma
      sigma(0) = std::max(-3.0, std::min(3.0, sigma(0)));
      sigma(1) = std::max(-3.0, std::min(3.0, sigma(1)));

      // Publish sigma (wheel velocities)
      cmd_vel.angular.x = sigma(0);
      cmd_vel.angular.y = sigma(1);
      
      // Publish cmd_vel
      wheel_vel_publisher->publish(cmd_vel);

      // Publish desired pose
      geometry_msgs::msg::PoseStamped world_desired_pose;
      world_desired_pose.header.stamp = this->now();
      world_desired_pose.header.frame_id = "world";
      world_desired_pose.pose.position.x = x_d_w(0);
      world_desired_pose.pose.position.y = x_d_w(1);
      world_desired_pose.pose.position.z = 0.0;
      world_desired_pose.pose.orientation.x = 0.0;
      world_desired_pose.pose.orientation.y = 0.0;
      world_desired_pose.pose.orientation.z = 0.0;
      world_desired_pose.pose.orientation.w = 1.0;

      world_desired_pose_publisher->publish(world_desired_pose);
    }

  private:
    float r;
    float l;
    float d;
    float kp;
    float kd;
    int robot_id;
    std::string robot_name;

    geometry_msgs::msg::Twist cmd_vel;

    Eigen::Matrix2d A;
    Eigen::Vector2d u;
    Eigen::Vector3d x_d_l;
    Eigen::Vector3d x_d_w;
    Eigen::Vector3d leader_pose;
    Eigen::Vector3d leader_velocity;
    Eigen::Vector3d x_d_l_dot;
    Eigen::Vector3d x_d_w_dot;
    Eigen::Vector2d uaux;
    Eigen::Vector3d x;
    Eigen::Vector2d e;
    Eigen::Vector2d e_prev;
    Eigen::Vector2d e_dot;
    Eigen::Vector2d sigma;
    Eigen::Vector2d sigma_prev;

    rclcpp::TimerBase::SharedPtr control_timer; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimator_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leader_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr leader_velocity_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr world_desired_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_vel_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationAsmc>());
  rclcpp::shutdown();
  return 0;
}
