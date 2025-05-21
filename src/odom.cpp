#include <map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "aruco_interfaces/msg/aruco_markers.hpp"
#include "std_msgs/msg/bool.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/QR>
#include "std_msgs/msg/float32.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class PuzzlebotOdom : public rclcpp::Node{
  public:
    PuzzlebotOdom(): Node("odom_node"){
      // Get puzzlebot id from launchfile parameter
      this->declare_parameter("puzzlebot_id", 0);
      this->get_parameter("puzzlebot_id", puzzlebot_id);

      this->declare_parameter("use_prefix", false);
      this->get_parameter("use_prefix", use_prefix);

      global_poses = YAML::LoadFile(ament_index_cpp::get_package_share_directory("aphelion") + "/config/marker_poses.yaml");

      for(const auto& node : global_poses) {
        world_poses[node["id"].as<int>()] = {node["x"].as<double>(), node["id"].as<double>()};
      }

      // Subscribers
      if (use_prefix){
        estimator_pose_publisher     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/estimator/pose", 10);
        estimator_velocity_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/estimator/velocity", 10);
	odometry_publisher           = this->create_publisher<nav_msgs::msg::Odometry>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/odometry", 10);
        wheel_l_subscriber           = this->create_subscription<std_msgs::msg::Float32>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/wL", 10, std::bind(&PuzzlebotOdom::wheel_l_callback, this, std::placeholders::_1));
        wheel_r_subscriber           = this->create_subscription<std_msgs::msg::Float32>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/wR", 10, std::bind(&PuzzlebotOdom::wheel_r_callback, this, std::placeholders::_1));
        aruco_subscriber             = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>("/puzzlebot_" + std::to_string(puzzlebot_id) + "/aruco_markers", 10, std::bind(&PuzzlebotOdom::vision_callback, this, std::placeholders::_1));
      } else {
        estimator_pose_publisher     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimator/pose", 10);
        estimator_velocity_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/estimator/velocity", 10);
	odometry_publisher           = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
        wheel_l_subscriber           = this->create_subscription<std_msgs::msg::Float32>("/wL", 10, std::bind(&PuzzlebotOdom::wheel_l_callback, this, std::placeholders::_1));
        wheel_r_subscriber           = this->create_subscription<std_msgs::msg::Float32>("/wR", 10, std::bind(&PuzzlebotOdom::wheel_r_callback, this, std::placeholders::_1));
        aruco_subscriber             = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&PuzzlebotOdom::vision_callback, this, std::placeholders::_1));
      }

      control_timer                  = this->create_wall_timer(10ms, std::bind(&PuzzlebotOdom::control_callback, this));
      tf_broadcaster                 = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      // Initialize variables
      r = 0.1; // Wheel radius
      l = 0.5; // Wheel distance

      x_hat          << 0.0, 0.0, 0.0; // Initial state
      x_hat_dot      << 0.0, 0.0, 0.0;
      x_hat_dot_prev << 0.0, 0.0, 0.0;
      u              << 0.0, 0.0;
      P << 0,0,0,
           0,0,0,
           0,0,0;

      Q << 0.1, 0, 0,
           0, 0.1, 0,
           0, 0, 0.1;
      
      R << 0.1, 0,
           0, 0.1;

    }

    void wheel_l_callback(const std_msgs::msg::Float32::SharedPtr msg){
      u(0) = msg->data;
    }

    void wheel_r_callback(const std_msgs::msg::Float32::SharedPtr msg){
      u(1) = msg->data;
    }

    void control_callback(){
      // Define A matrix
      A << (r/2)*cos(x_hat(2)), (r/2)*cos(x_hat(2)),
           (r/2)*sin(x_hat(2)), (r/2)*sin(x_hat(2));

      // Compute x_hat_dot
      helper = A * u;
      x_hat_dot(0) = helper(0);
      x_hat_dot(1) = helper(1);
      x_hat_dot(2) = r * ((u(1) - u(0)) / l); 

      F << 1, 0, -sin(x_hat(2)) * (u(0) + u(1)) * r / 2 * 0.01,
           0, 1, cos(x_hat(2)) * (u(0) + u(1)) * r / 2 * 0.01,
           0, 0, 1;
      
      P = F * P * F.transpose() + Q;
      
      // Trapezoidal integral of x_hat_dot
      x_hat += 0.01 * 0.5 * (x_hat_dot + x_hat_dot_prev);
      x_hat_dot_prev = x_hat_dot;

      // Wrap yaw
      x_hat(2) = fmod(x_hat(2) + M_PI, 2*M_PI) - M_PI;

      

      // Publish
      estimator_velocity_msg.twist.linear.x  = x_hat_dot(0);
      estimator_velocity_msg.twist.linear.y  = x_hat_dot(1);
      estimator_velocity_msg.twist.angular.z = x_hat_dot(2);

      estimator_velocity_msg.header.stamp = this->now();
      estimator_velocity_msg.header.frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";

      estimator_pose_msg.pose.position.x = x_hat(0);
      estimator_pose_msg.pose.position.y = x_hat(1);

      Eigen::Quaterniond q(Eigen::AngleAxisd(x_hat(2), Eigen::Vector3d::UnitZ()));
      estimator_pose_msg.pose.orientation.x = q.x();
      estimator_pose_msg.pose.orientation.y = q.y();
      estimator_pose_msg.pose.orientation.z = q.z();
      estimator_pose_msg.pose.orientation.w = q.w();

      estimator_pose_msg.header.stamp = this->now();
      // estimator_pose_msg.header.frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      estimator_pose_msg.header.frame_id = "odom";
  
      odom_msg.header.stamp = this->now();
      // odom_msg.header.frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_footprint";
      odom_msg.pose.pose = estimator_pose_msg.pose;
      odom_msg.twist.twist = estimator_velocity_msg.twist;
      // Diagonal covariance
      odom_msg.pose.covariance[0] = P(0, 0);
      odom_msg.pose.covariance[1] = P(0, 1);
      odom_msg.pose.covariance[5] = P(0, 2);
      odom_msg.pose.covariance[6] = P(1, 0);
      odom_msg.pose.covariance[7] = P(1, 1);
      odom_msg.pose.covariance[11] = P(1, 2);
      odom_msg.pose.covariance[30] = P(2, 0);
      odom_msg.pose.covariance[31] = P(2, 1);
      odom_msg.pose.covariance[35] = P(2, 2);
      

      // Publish transform
      
      //odom_transform_msg.header.frame_id = "world";
      //odom_transform_msg.child_frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      odom_transform_msg.header.frame_id = "map";
      odom_transform_msg.child_frame_id = "odom";
      odom_transform_msg.header.stamp = this->now();
      odom_transform_msg.transform.translation.x = 0.0;
      odom_transform_msg.transform.translation.y = 0.0;
      odom_transform_msg.transform.rotation.x = 0.0;
      odom_transform_msg.transform.rotation.y = 0.0;
      odom_transform_msg.transform.rotation.z = 0.0;
      odom_transform_msg.transform.rotation.w = 1.0;

      tf_broadcaster->sendTransform(odom_transform_msg);

      odom_transform_msg.header.stamp = this->now();
      //odom_transform_msg.header.frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      odom_transform_msg.child_frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_base_footprint";
      odom_transform_msg.header.frame_id = "odom";
      // odom_transform_msg.child_frame_id = "base_footprint";
      
      odom_transform_msg.transform.translation.x = x_hat(0);
      odom_transform_msg.transform.translation.y = x_hat(1);
      odom_transform_msg.transform.rotation.x = q.x();
      odom_transform_msg.transform.rotation.y = q.y();
      odom_transform_msg.transform.rotation.z = q.z();
      odom_transform_msg.transform.rotation.w = q.w();

      tf_broadcaster->sendTransform(odom_transform_msg);

      estimator_velocity_publisher->publish(estimator_velocity_msg);
      estimator_pose_publisher->publish(estimator_pose_msg);
      odometry_publisher->publish(odom_msg);
    }

    void vision_callback(const aruco_interfaces::msg::ArucoMarkers &markers) {
      size_t i;
      geometry_msgs::msg::Pose aruco_pose;
      int aruco_id;
      for(i = 0; i < markers.marker_ids.size(); i++) {
        aruco_id = markers.marker_ids.at(i);
        aruco_pose = markers.poses.at(i);
        z_hat << sin(x_hat(2)) * (world_poses[aruco_id][0] - x_hat(0)) - cos(x_hat(2)) * (world_poses[aruco_id][1] - x_hat(1)),
                 cos(x_hat(2)) * (world_poses[aruco_id][0] - x_hat(0)) + sin(x_hat(2)) * (world_poses[aruco_id][1] - x_hat(1)) - 0.1;
        
        H << -sin(x_hat(2)), cos(x_hat(2)), cos(x_hat(2)) * (world_poses[aruco_id][0] - x_hat(0)) + sin(x_hat(2)) * (world_poses[aruco_id][1] - x_hat(1)),
             -cos(x_hat(2)), -sin(x_hat(2)), -sin(x_hat(2)) * (world_poses[aruco_id][0] - x_hat(0)) + cos(x_hat(2)) * (world_poses[aruco_id][1] - x_hat(1));
        
        y_hat << aruco_pose.position.x - z_hat(0),
                 aruco_pose.position.z - z_hat(1);
        
        S = H * P * H.transpose() + R;
        K = P * H.transpose() * S.inverse();

        x_hat += K * y_hat;
        P = (Eigen::Matrix3d::Identity(3, 3) - K * H) * P;
      }


      estimator_pose_msg.pose.position.x = x_hat(0);
      estimator_pose_msg.pose.position.y = x_hat(1);

      Eigen::Quaterniond q(Eigen::AngleAxisd(x_hat(2), Eigen::Vector3d::UnitZ()));
      estimator_pose_msg.pose.orientation.x = q.x();
      estimator_pose_msg.pose.orientation.y = q.y();
      estimator_pose_msg.pose.orientation.z = q.z();
      estimator_pose_msg.pose.orientation.w = q.w();

      estimator_pose_msg.header.stamp = this->now();
      // estimator_pose_msg.header.frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      estimator_pose_msg.header.frame_id = "odom";
  
      odom_msg.header.stamp = this->now();
      // odom_msg.header.frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_footprint";
      odom_msg.pose.pose = estimator_pose_msg.pose;
      // Diagonal covariance
      odom_msg.pose.covariance[0] = P(0, 0);
      odom_msg.pose.covariance[1] = P(0, 1);
      odom_msg.pose.covariance[5] = P(0, 2);
      odom_msg.pose.covariance[6] = P(1, 0);
      odom_msg.pose.covariance[7] = P(1, 1);
      odom_msg.pose.covariance[11] = P(1, 2);
      odom_msg.pose.covariance[30] = P(2, 0);
      odom_msg.pose.covariance[31] = P(2, 1);
      odom_msg.pose.covariance[35] = P(2, 2);
            // Publish transform
      
      //odom_transform_msg.header.frame_id = "world";
      //odom_transform_msg.child_frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      odom_transform_msg.header.frame_id = "map";
      odom_transform_msg.child_frame_id = "odom";
      odom_transform_msg.header.stamp = this->now();
      odom_transform_msg.transform.translation.x = 0.0;
      odom_transform_msg.transform.translation.y = 0.0;
      odom_transform_msg.transform.rotation.x = 0.0;
      odom_transform_msg.transform.rotation.y = 0.0;
      odom_transform_msg.transform.rotation.z = 0.0;
      odom_transform_msg.transform.rotation.w = 1.0;

      tf_broadcaster->sendTransform(odom_transform_msg);

      odom_transform_msg.header.stamp = this->now();
      //odom_transform_msg.header.frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_odom";
      odom_transform_msg.child_frame_id = "puzzlebot_" + std::to_string(puzzlebot_id) + "_base_footprint";
      odom_transform_msg.header.frame_id = "odom";
      // odom_transform_msg.child_frame_id = "base_footprint";
      
      odom_transform_msg.transform.translation.x = x_hat(0);
      odom_transform_msg.transform.translation.y = x_hat(1);
      odom_transform_msg.transform.rotation.x = q.x();
      odom_transform_msg.transform.rotation.y = q.y();
      odom_transform_msg.transform.rotation.z = q.z();
      odom_transform_msg.transform.rotation.w = q.w();

      tf_broadcaster->sendTransform(odom_transform_msg);

      estimator_velocity_publisher->publish(estimator_velocity_msg);
      estimator_pose_publisher->publish(estimator_pose_msg);
      odometry_publisher->publish(odom_msg);
    }

  private:
    geometry_msgs::msg::TwistStamped     estimator_velocity_msg;
    geometry_msgs::msg::PoseStamped      estimator_pose_msg;
    geometry_msgs::msg::TransformStamped odom_transform_msg;
    nav_msgs::msg::Odometry              odom_msg;

    Eigen::Matrix2d A, R, S;
    Eigen::Matrix3d P, F, Q;
    Eigen::Matrix<double, 2, 3> H;
    Eigen::Matrix<double, 3, 2> K;
    Eigen::Vector2d u, h, z_hat, y_hat;
    Eigen::Vector3d x_hat;
    Eigen::Vector3d x_hat_dot;
    Eigen::Vector3d x_hat_dot_prev;
    Eigen::Vector2d helper;
    float r;
    float l;
    bool use_prefix;
    int puzzlebot_id;

    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        wheel_l_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr        wheel_r_subscriber;
    rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr        aruco_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  estimator_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr estimator_velocity_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odometry_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    YAML::Node global_poses;
    std::map<int, std::vector<double>> world_poses;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PuzzlebotOdom>());
  rclcpp::shutdown();
  return 0;
}
