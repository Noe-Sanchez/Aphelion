#include <map>
#include <future>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "aphelion_msgs/msg/pose_command.hpp"
#include "aphelion_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
  FutureT & future,
  WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class MasterControl : public rclcpp::Node{
  public:
    MasterControl(): Node("master_control") {
      this->declare_parameter("asmc_node_name", "asmc_node");
      this->get_parameter(    "asmc_node_name",  asmc_node_name);
      
      this->declare_parameter("palign_node_name", "pallet_align_node");
      this->get_parameter(    "palign_node_name",  palign_node_name);

      navigation_state = aphelion_msgs::msg::NavState::STATE_STOP; // Start in stop state
      transitioning = false; 
      
      asmc_transitions_sub   = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("/" + asmc_node_name + "/transition_event", 10, std::bind(&MasterControl::astar_trans_callback, this, std::placeholders::_1));
      palign_transitions_sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("/" + palign_node_name + "/transition_event", 10, std::bind(&MasterControl::astar_trans_callback, this, std::placeholders::_1));
      
      asmc_change_state   = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + asmc_node_name + "/change_state");
      palign_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + palign_node_name + "/change_state");

      state_override_sub = this->create_subscription<aphelion_msgs::msg::NavState>("/master/state_override", 10, std::bind(&MasterControl::state_override_callback, this, std::placeholders::_1));
      integration_sub    = this->create_subscription<std_msgs::msg::String>("/integration", 10, std::bind(&MasterControl::integration_callback, this, std::placeholders::_1));

      // Pubs and subs for orchestra
      desired_pose_pub = this->create_publisher<aphelion_msgs::msg::PoseCommand>("/desired_pose", 10);
      odometry_sub     = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&MasterControl::odometry_callback, this, std::placeholders::_1));
      goal_pose_pub    = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10); 

      state_machine_timer = this->create_wall_timer(100ms, std::bind(&MasterControl::state_machine_callback, this));
 
      pallet_pose.header.frame_id = "map";
      truck_pose.header.frame_id  = "map";

      pallet_pose.pose.position.x = 0.6;
      pallet_pose.pose.position.y = 0.45;
      pallet_pose.pose.position.z = 0.0;
 
      pallet_pose.pose.orientation.x = 0.0;
      pallet_pose.pose.orientation.y = 0.0;
      pallet_pose.pose.orientation.z = 0.71;
      pallet_pose.pose.orientation.w = 0.71;
      
      truck_pose.pose.position.x = 0.0;
      truck_pose.pose.position.y = 0.0;
      truck_pose.pose.position.z = 0.0;

      truck_pose.pose.orientation.x = 0.0;
      truck_pose.pose.orientation.y = 0.0;
      truck_pose.pose.orientation.z = 0.0;
      truck_pose.pose.orientation.w = 1.0;
      
    }

    bool change_state(std::uint8_t transition, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client, std::chrono::seconds time_out = 3s) {
      auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      request->transition.id = transition;

      if (!client->wait_for_service(time_out)) {
        RCLCPP_ERROR(
          get_logger(),
          "Service %s is not available.",
          client->get_service_name());
        return false;
      }

      // We send the request with the transition we want to invoke.
      RCLCPP_INFO(get_logger(), "Requesting state change");

      // Just send request, dont wait for response
      auto future = client->async_send_request(request);
      // Wait for the result with a timeout
      auto status = wait_for_result(future, 100ms);
      if (status != std::future_status::ready) {
	RCLCPP_INFO(get_logger(), "Service call timed out after %ld seconds", time_out.count());
      }

      return true;
    }

    void asmc_trans_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
      
    }

    void astar_trans_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
      
    }

    void integration_callback(const std_msgs::msg::String::SharedPtr msg){
      RCLCPP_INFO(get_logger(), "Someone finished: %s", msg->data);
      integration_msg = *msg;
    }

    void state_override_callback(const aphelion_msgs::msg::NavState::SharedPtr msg){
      RCLCPP_INFO(get_logger(), "Received state override from (%d) to (%d)", navigation_state, msg->state);
      navigation_state = msg->state; 
      transitioning = true; 
    } 

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
      current_odometry = msg;
    } 

    void state_machine_callback(){
      
      // Switch states
      switch ( navigation_state ) {
	case aphelion_msgs::msg::NavState::STATE_STOP:{
	  if ( transitioning ){
	    while ( true ){
	      // Ask for state transition
	      bool req = change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, asmc_change_state); 
	      if ( req ) {
		RCLCPP_INFO(get_logger(), "Stopping...");
		break; 
	      } else {
		RCLCPP_ERROR(get_logger(), "Failed to change state to ASTAR, retrying...");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	      }
	    }

	    transitioning = false;
	  }

	  RCLCPP_INFO(get_logger(), "Stopped");
	  break;
        }
	case aphelion_msgs::msg::NavState::STATE_ASTAR_PALLET:{
	  // Transition clause
	  if ( transitioning ){
	    while ( true ){
	      // Ask for state transition
	      bool req = change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, asmc_change_state); 
	      if ( req ) {
		RCLCPP_INFO(get_logger(), "Going ASTAR to Pallet...");
		
		pallet_pose.header.stamp = this->now();

		goal_pose_pub->publish(pallet_pose);	

		break; 
	      } else {
		RCLCPP_ERROR(get_logger(), "Failed to change state to ASTAR, retrying...");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	      }
	    }

	    transitioning = false;
	  }

	  RCLCPP_INFO(get_logger(), "Going to goal");

	  if ( integration_msg.data == "ASTAR" ){
	    RCLCPP_INFO(get_logger(), "ASTAR to pallet finished, aligning..."); 
	    navigation_state = aphelion_msgs::msg::NavState::STATE_PALLET_ALIGN; 
	    transitioning = true;
	  }
	  
	  break; 
	}
	case aphelion_msgs::msg::NavState::STATE_PALLET_ALIGN:{
	  // Transition clause
	  if ( transitioning ){
	    while ( true ){
	      // Ask for state transition
	      bool req = change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, palign_change_state); 
	      if ( req ) {
		RCLCPP_INFO(get_logger(), "Aligning to Pallet...");

		break; 
	      } else {
		RCLCPP_ERROR(get_logger(), "Failed to change state to PALIGN, retrying...");
		std::this_thread::sleep_for(std::chrono::seconds(1));
	      }
	    }

	    transitioning = false;
	  }

	  RCLCPP_INFO(get_logger(), "Aligning");

	  if ( integration_msg.data == "PALIGN" ){
	    RCLCPP_INFO(get_logger(), "PALIGN to pallet finished, picking..."); 
	    navigation_state = aphelion_msgs::msg::NavState::STATE_PALLET_PICK;
	    transitioning = true;
	  }
	  
	  break; 
	}
	case aphelion_msgs::msg::NavState::STATE_PALLET_PICK:{
	  RCLCPP_INFO(get_logger(), "All for now, folks");
	  break; 
	}
	default:{
	  RCLCPP_ERROR(get_logger(), "State %d defaulted, is everything ok?", navigation_state);
	  break;
	}
      }
    }

  private:
    uint8_t     navigation_state; 
    std::string asmc_node_name, palign_node_name;
    bool        transitioning;
    
    std_msgs::msg::String                      integration_msg;
    nav_msgs::msg::Odometry::SharedPtr         current_odometry;
    aphelion_msgs::msg::PoseCommand::SharedPtr desired_pose;

    geometry_msgs::msg::PoseStamped pallet_pose;
    geometry_msgs::msg::PoseStamped truck_pose;

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr           asmc_change_state,    palign_change_state;
    
    rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr asmc_transitions_sub, palign_transitions_sub;
    rclcpp::Subscription<aphelion_msgs::msg::NavState>::SharedPtr         state_override_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              odometry_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                integration_sub; 

    rclcpp::Publisher<aphelion_msgs::msg::PoseCommand>::SharedPtr         desired_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr         goal_pose_pub;

    rclcpp::TimerBase::SharedPtr state_machine_timer;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MasterControl>());

  // Create multithreaded executor
  //rclcpp::executors::MultiThreadedExecutor executor;
  //auto master_control_node = std::make_shared<MasterControl>();
  //executor.add_node(master_control_node);
  //executor.spin();
  //

  auto node = std::make_shared<MasterControl>();


  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();  

  rclcpp::shutdown();
  return 0;
}
