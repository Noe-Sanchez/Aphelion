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
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <eigen3/Eigen/QR>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "aphelion_msgs/msg/pose_command.hpp"
#include "aruco_interfaces/msg/aruco_markers.hpp"


using namespace std::chrono_literals;

double fsign(double x) {
  if (x > 0.0){ return 1.0; }
  else if (x < 0.0){ return -1.0; }
  else{ return 0.0; }
}

//class PalletAlign : public rclcpp::Node{
class PalletAlign : public rclcpp_lifecycle::LifecycleNode{ 
  public:
    //PalletAlign(): Node("pallet_alignment_node"){
    PalletAlign(): rclcpp_lifecycle::LifecycleNode("pallet_align_node"){ 
    
      // Variables?
      x << 0, 0, 0;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Configuring Pallet Alignment Node");

      estimator_pose_subscriber      = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&PalletAlign::estimator_pose_callback, this, std::placeholders::_1));
      aruco_poses_subscriber         = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&PalletAlign::aruco_markers_callback, this, std::placeholders::_1));
      desired_pose_publisher         = this->create_publisher<aphelion_msgs::msg::PoseCommand>("/desired_pose", 10);
      marker_visualization_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/marker_visualization", 10);
      marker_visualization_publisher2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("/marker_visualization2", 10);

      integration_publisher = this->create_publisher<std_msgs::msg::String>("/integration",10);
      

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Activating Pallet Alignment Node");

      // Check if we have the aruco markers
      if (aruco_markers.poses.size() == 0){
      	RCLCPP_WARN(this->get_logger(), "No Aruco markers detected");
      	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
      }
      
      // Get aruco marker with ID 14
      bool aruco_found = false;
      for ( int i = 0; i < aruco_markers.poses.size(); i++ ){
	if ( aruco_markers.marker_ids[i] == 14 ){
	  target_marker = aruco_markers.poses[i];
	  aruco_found = true;
	  break;
	}
      }
      if ( !aruco_found ) { return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR; }
      
      // Start the timer
      // Activate publisher
      desired_pose_publisher->on_activate();
      //control_timer             = this->create_wall_timer(100ms, std::bind(&PalletAlign::control_callback, this));
      control_timer             = this->create_wall_timer(100ms, std::bind(&PalletAlign::control_callback, this));

      // Calculate the desired pose
      /*desired_pose.header.stamp = this->now();
      desired_pose.header.frame_id = "map"; 


       Calculate joint quaternion
      Eigen::Quaterniond q_robot;
      q_robot.x() = current_odometry.pose.pose.orientation.x;
      q_robot.y() = current_odometry.pose.pose.orientation.y;
      q_robot.z() = current_odometry.pose.pose.orientation.z;
      q_robot.w() = current_odometry.pose.pose.orientation.w;

      Eigen::Quaterniond q_marker;
      q_marker.x() = target_marker.pose.orientation.x;
      q_marker.y() = target_marker.pose.orientation.y;
      q_marker.z() = target_marker.pose.orientation.z;
      q_marker.w() = target_marker.pose.orientation.x;

      // Set position
      desired_pose.position.x = x(0) + marker.pose.position.z;
      desired_pose.position.y = x(1) - marker.pose.position.y;
      desired_pose.position.z = 0.0; // Assuming planar robot
      
      // Set orientation (facing towards the marker)
      tf2::Quaternion q;
      q.setRPY(0, 0, x(2)); // Keep current yaw
      desired_pose.orientation.x = q.x();
      desired_pose.orientation.y = q.y();
      desired_pose.orientation.z = q.z();
      desired_pose.orientation.w = q.w();
      
      // Publish the desired pose
      desired_pose_publisher->publish(desired_pose);*/

      Eigen::Quaterniond q_robot;
      q_robot.x() = current_odometry.pose.pose.orientation.x;
      q_robot.y() = current_odometry.pose.pose.orientation.y;
      q_robot.z() = current_odometry.pose.pose.orientation.z;
      q_robot.w() = current_odometry.pose.pose.orientation.w;

      Eigen::Quaterniond q_marker;
      q_marker.x() =  target_marker.orientation.w;
      q_marker.y() =  target_marker.orientation.y;
      q_marker.z() = -target_marker.orientation.x;
      q_marker.w() =  target_marker.orientation.z;

      // Apply odom quaternion to marker pose using Eigen
      //Eigen::Quaterniond target_vector( 0, target_marker.position.z, -target_marker.position.x, target_marker.position.y);
      //Eigen::Quaterniond target_vector( 0, target_marker.position.z, -target_marker.position.x, target_marker.position.y);
      Eigen::Quaterniond target_vector;
      target_vector.x() =  target_marker.position.z;
      target_vector.y() = -target_marker.position.x;
      target_vector.z() =  target_marker.position.y;
      target_vector.w() =  0;
      Eigen::Quaterniond target_vector2 = target_vector;
      //target_vector2.x() += 0.1;
      //target_vector2.y() -= 0.03;
      target_vector2.x() -= 0.35;
      target_vector2.y() -= 0.0775;
      target_vector.x()  += 0.025;
      target_vector.y()  -= 0.0775;

      Eigen::Quaterniond q_desired = q_robot * q_marker;
      //Eigen::Quaterniond q_pucha (0, 0, 0.71, 0.71);
      //q_desired = (q_pucha.inverse()) * q_desired;
      //target_vector = q_robot.inverse() * target_vector * q_robot;
      target_vector = q_robot * target_vector * q_robot.inverse();
      target_vector2 = q_robot * target_vector2 * q_robot.inverse();
      //q_desired = q_desired.inverse();
      //target_vector = q_desired * target_vector * q_desired.inverse();

      marker_visualization.header.stamp = this->now();
      marker_visualization.header.frame_id = "map";
      marker_visualization.pose.position.x = current_odometry.pose.pose.position.x + target_vector.x();
      marker_visualization.pose.position.y = current_odometry.pose.pose.position.y + target_vector.y();

      
      //Eigen::Quaterniond q_k  = q_desired.conjugate();
      marker_visualization.pose.orientation.x = q_robot.x();
      marker_visualization.pose.orientation.y = q_robot.y();
      marker_visualization.pose.orientation.z = q_robot.z();
      marker_visualization.pose.orientation.w = q_robot.w();
      //imarker_visualization.pose.orientation.x = q_k.x();
      //marker_visualization.pose.orientation.y = q_k.y();
      //marker_visualization.pose.orientation.z = q_k.z();
      //marker_visualization.pose.orientation.w = q_k.w();

      marker_visualization2 = marker_visualization;
      marker_visualization2.pose.position.x = current_odometry.pose.pose.position.x + target_vector2.x(); 
      marker_visualization2.pose.position.y = current_odometry.pose.pose.position.y + target_vector2.y();
      /*marker_visualization.pose.orientation.x = q_marker.x();
      marker_visualization.pose.orientation.y = q_marker.y();
      marker_visualization.pose.orientation.z = q_marker.z();
      marker_visualization.pose.orientation.w = q_marker.w();*/



      desired_pose.target_pose = marker_visualization2;
      desired_pose.target_pose.header.stamp = this->now();
      desired_pose.target_pose.header.frame_id = "map";
      desired_pose.do_realign = true;

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Deactivating Pallet Alignment Node");

      // Stop the timer
      // Deactivate publisher
      desired_pose_publisher->on_deactivate();
      control_timer->cancel();
      
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) {
      RCLCPP_INFO(this->get_logger(), "Cleaning up Pallet Alignment Node");
      
      // Unsubscribe from topics
      estimator_pose_subscriber.reset();
      aruco_poses_subscriber.reset();
      desired_pose_publisher.reset();
      
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void estimator_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){ 
      // Convert quaternion to euler angles (planar robot, so only yaw)
      //tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      /*tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      //x << msg->pose.pose.position.x,
      //	   msg->pose.pose.position.y,
      //	   yaw;

      // Apply transform to the odometry data
      x(0) = msg->pose.pose.position.x; 
      x(1) = msg->pose.pose.position.y;
      x(2) = yaw;*/ 
      current_odometry = *msg;

    }

    void aruco_markers_callback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
      aruco_markers = *msg; 
    }

    void control_callback(){ 
      
      marker_visualization_publisher->publish(marker_visualization);
      marker_visualization_publisher2->publish(marker_visualization2);

      // Check euclidean distance to the desired pose
      //double distance = sqrt(pow(marker_visualization.pose.position.x - current_odometry.pose.pose.position.x, 2) + 
      //	       	       pow(marker_visualization.pose.position.y - current_odometry.pose.pose.position.y, 2));
      double distance = sqrt(pow(desired_pose.target_pose.pose.position.x - current_odometry.pose.pose.position.x, 2) + 
			     pow(desired_pose.target_pose.pose.position.y - current_odometry.pose.pose.position.y, 2));

      if ( first_alignment ){
	if ( distance > 0.0275 ){
	  desired_pose.target_pose = marker_visualization2;
	  desired_pose.target_pose.header.stamp = this->now();
	}else{
	  // Sleep node for 2 seconds
	  RCLCPP_INFO(this->get_logger(), "Pallet Alignment Complete");
	  rclcpp::sleep_for(std::chrono::seconds(5));

	  desired_pose.target_pose = marker_visualization;
	  desired_pose.target_pose.header.stamp = this->now();
	  first_alignment = false;
	}
      }else{
	if ( distance < 0.0275 ){
	  // Were done, deactivate the node
	  RCLCPP_INFO(this->get_logger(), "Pallet Pick Complete");

	  std_msgs::msg::String end_msg;
	  end_msg.data = "PALIGN";
          integration_publisher->publish(end_msg);
	  
	  first_alignment = true;

	  this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
	  return;
	}
      }

      desired_pose_publisher->publish(desired_pose);

    }

  private:
    aruco_interfaces::msg::ArucoMarkers aruco_markers;
    aphelion_msgs::msg::PoseCommand     desired_pose;
    geometry_msgs::msg::PoseStamped     marker_visualization;
    geometry_msgs::msg::PoseStamped     marker_visualization2;
    geometry_msgs::msg::Pose            target_marker;
    nav_msgs::msg::Odometry             current_odometry;

    Eigen::Vector3d x;

    bool first_alignment = true;

    rclcpp::TimerBase::SharedPtr control_timer; 
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desired_pose_publisher;
    
    rclcpp_lifecycle::LifecyclePublisher<aphelion_msgs::msg::PoseCommand>::SharedPtr desired_pose_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                         estimator_pose_subscriber;
    rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr             aruco_poses_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_visualization_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_visualization_publisher2;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr integration_publisher;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<PalletAlign> pallet_alignment_node = std::make_shared<PalletAlign>();
  executor.add_node(pallet_alignment_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  //rclcpp::spin(std::make_shared<PalletAlign>());

  return 0;
}
