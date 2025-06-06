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

#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

enum navigationState{
    STANDBY,
    ASTAR,
    PALLET_ALIGN,
    PALLET_GRASP,
    TRUCK_ALIGN,
    TRUCK_GRASP,
    STOP
};

class MasterControl : public rclcpp::Node{
  public:
    MasterControl(): Node("master_control") {
      this->declare_parameter("asmc_node_name",  "asmc_node");
      this->get_parameter(    "asmc_node_name",   asmc_node_name);
      this->declare_parameter("astar_node_name", "guild_navigator_node");
      this->get_parameter(    "astar_node_name",  astar_node_name);

      curr_state = STOP; 

      //lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan2",  10, std::bind(&MasterControl::lidar_callback, this, std::placeholders::_1));
      min_dist_sub = this->create_subscription<std_msgs::msg::Float32>("/min_distance", 10, std::bind(&MasterControl::min_dist_callback, this, std::placeholders::_1));
      
      bug0_transitions_sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("/" + bug0_node_name + "/transition_event", 10, std::bind(&MasterControl::astar_trans_callback, this, std::placeholders::_1));
      asmc_transitions_sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("/" + asmc_node_name + "/transition_event", 10, std::bind(&MasterControl::astar_trans_callback, this, std::placeholders::_1));
      astar_transitions_sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("/" + astar_node_name + "/transition_event", 10, std::bind(&MasterControl::astar_trans_callback, this, std::placeholders::_1));
      
      
      bug0_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + bug0_node_name + "/change_state");
      asmc_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + asmc_node_name + "/change_state");
      astar_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + astar_node_name + "/change_state");
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
      auto future_result = client->async_send_request(request).future.share();

      // Let's wait until we have the answer from the node.
      // If the request times out, we return an unknown state.
      auto future_status = wait_for_result(future_result, time_out);

      if (future_status != std::future_status::ready) {
        RCLCPP_ERROR(
          get_logger(), "Server time out while getting current state for node");
        return false;
      }

      // We have an answer, let's print our success.
      if (future_result.get()->success) {
        RCLCPP_INFO(
          get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
        return true;
      } else {
        RCLCPP_WARN(
          get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
        return false;
      }
    }

    void min_dist_callback(const std_msgs::msg::Float32::SharedPtr msg){
      RCLCPP_INFO(get_logger(), "Current min distance: %f", msg->data);
      if (msg->data < 0.3){
        if(curr_state ==  ASTAR){
	  change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, bug0_change_state);
          change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, astar_change_state);
          curr_state = BUG0;
	  RCLCPP_INFO(get_logger(), "BUG0 IS ACTIVATED");
	  return;
          }
      }else{
	if (curr_state == BUG0){
	  change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, bug0_change_state);
	  change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, astar_change_state);
	  curr_state = ASTAR;
	  RCLCPP_INFO(get_logger(), "ASTAR IS ACTIVATED");
	}
      }
    }
    
   
    /*
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
      for(double range: msg->ranges){
        if(range < 0.5){
          if(curr_state ==  ASTAR){
            change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, bug0_change_state);
            change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, astar_change_state);
            curr_state = BUG0;
            return;
          }
        }
      }
      if(curr_state == BUG0){
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, bug0_change_state);
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, astar_change_state);
        curr_state = ASTAR;
      }
    }*/

    void bug0_trans_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
      
    }

    void asmc_trans_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
      
    }

    void astar_trans_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg){
      
    }

  private:
    std::string asmc_node_name, bug0_node_name, astar_node_name;
    navigationState curr_state;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr min_dist_sub;
    rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr asmc_transitions_sub, bug0_transitions_sub, astar_transitions_sub;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr asmc_change_state, bug0_change_state, astar_change_state;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterControl>());
  rclcpp::shutdown();
  return 0;
}
