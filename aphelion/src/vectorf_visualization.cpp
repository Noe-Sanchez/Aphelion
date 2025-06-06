#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>
#include "mrsl_math.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class VectorFViz : public rclcpp::Node{
  public:
    VectorFViz(): Node("vectorfviz_node"){
      //reference_velocity_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped>("tello/reference/velocity", 10, std::bind(&VectorFViz::velocity_reference_callback, this, std::placeholders::_1));
      //uaux_publisher    = this->create_publisher<geometry_msgs::msg::Twist>("tello/control/uaux", 10);

      viz_timer = this->create_wall_timer(50ms, std::bind(&VectorFViz::viz_callback, this));
      //viz_timer = this->create_wall_timer(500ms, std::bind(&VectorFViz::viz_callback, this));
  
      // Declare num_drones parameter
      this->declare_parameter("num_drones", 1);
      this->get_parameter("num_drones", num_drones);

      // Declare vf parameters
      xextent = 0.5;
      yextent = 0.5;
      zextent = 0.5;
      xdivs = 7;
      ydivs = 7;
      zdivs = 7;

      //D = xextent * 1.25;
      D = 0.5;

      // Estimate number of markers with xdivs, ydivs, zdivs and xextent
      num_markers = xdivs * ydivs * zdivs;

      // Fill drone_positions with zeros 
      for (int i = 0; i < num_markers; i++){
	drone_positions.push_back(Eigen::Vector3d(0, 0, 0));
      }

      // Declare drone pose subscriptions
      for(int i = 0; i < num_drones; i++){
	std::function <void(const geometry_msgs::msg::PoseStamped::SharedPtr)> cb = std::bind(&VectorFViz::drone_pos_callback, this, std::placeholders::_1, i);
	drone_pose_subscriptions.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>("tello_" + std::to_string(i) + "/tello/estimator/pose", 10, cb));
	drone_poses.push_back(geometry_msgs::msg::PoseStamped());
	drone_vec_publishers.push_back(this->create_publisher<visualization_msgs::msg::MarkerArray>("tello_" + std::to_string(i) + "/tello/viz/field", 10));
	drone_vecs.push_back(visualization_msgs::msg::MarkerArray());
      } 
    }

    void drone_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int index){
      drone_poses[index].pose = msg->pose;
    }

    void viz_callback(){
      this->get_parameter("num_drones", num_drones);
      if(c < 20){
	c += 0.5;
      }else{
	c = 0;
      }
      for (int i = 0; i < num_drones; i++){      
	drone_vecs[i].markers.clear();
	int id = 0;
	for (int j = 0; j < xdivs; j++){
	  for (int k = 0; k < ydivs; k++){
	    for (int l = 0; l < zdivs; l++){
	      visualization_msgs::msg::Marker marker;
	      // Check if element of drone_positions exists for id
	      float xpos, ypos, zpos;
	      /*if(id < (int)drone_positions.size()){
		std::cout << "Using existing interp for id" << id << std::endl;
		*/
		xpos = -xextent + 2*j*xextent/(xdivs-1) + drone_positions[id](0)*c*0.005;
		ypos = -yextent + 2*k*yextent/(ydivs-1) + drone_positions[id](1)*c*0.005;
		zpos = -zextent + 2*l*zextent/(zdivs-1) + drone_positions[id](2)*c*0.005;
	      //}else{
		//std::cout << "Using new interp for id" << id << std::endl;
		//xpos = -xextent + j*xextent/(xdivs-1);
		//ypos = -yextent + k*yextent/(ydivs-1);
		//zpos = -zextent + l*zextent/(zdivs-1);
	      //}*/
	      //float ypos = -yextent + k*yextent/(ydivs-1);
	      //float zpos = -zextent + l*zextent/(zdivs-1);
	      // Check magnitude of position: should we ignore this marker?
	      float mag = sqrt(xpos*xpos + ypos*ypos + zpos*zpos);
	      if (mag > D-0.01){
		marker.color.a = 0.01; 
	      }else{
		marker.color.a = 0.75;
	      }
	      marker.pose.position.x = xpos; 
	      marker.pose.position.y = ypos;
	      marker.pose.position.z = zpos;
	      marker.header.frame_id = "tello" + std::to_string(i);
	      //marker.header.stamp = this->now();
	      marker.type = visualization_msgs::msg::Marker::ARROW;
	      marker.action = visualization_msgs::msg::Marker::ADD;
	      //marker.scale.x = 0.25;
	      //marker.scale.y = 0.025;
	      //marker.scale.z = 0.025;
	      marker.scale.x = xextent/(xdivs-1);
	      marker.scale.y = xextent/((xdivs-1)*10);
	      marker.scale.z = xextent/((xdivs-1)*10);
	      //marker.color.a = 0.75;
	      // Change color depending on drone (i variable)
	      marker.color.r = ((float)i)/((float)num_drones-1); 
	      marker.color.g = 0.2;
	      marker.color.b = 1 - ((float)i)/((float)num_drones-1); 
	      //std::cout << "num_drones: " << num_drones << " i: " << i << " r: " << marker.color.r << " g: " << marker.color.g << " b: " << marker.color.b << std::endl;
	      // use j, k, l to set the position of the marker
	      // Use each axis for the vector field
	      Eigen::Vector3d pos;
	      //Eigen::Vector3d unit;
	      //unit << 1, 0, 0;
	      //if ( i == 0){
		
	      //pos << xpos - ypos, xpos + ypos, zpos;
	      
	      //}else{
	
	      pos << (-tanh((5/D)*abs(zpos))+1)*(xpos - ypos), 
		     (-tanh((5/D)*abs(zpos))+1)*(xpos + ypos),
		       zpos;
	      
	      //}
	      //pos << 0, 0, 1;
	      //std::cout << "id: " << id << " x: " << marker.pose.position.x << " y: " << marker.pose.position.y << " z: " << marker.pose.position.z << " ax: " << pos(0) << " ay: " << pos(1) << " az: " << pos(2) << std::endl;
	      //Normalize pos
	      pos = pos / sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
	      /*if(id < (int)drone_positions.size()){ 
	       */
		//std::cout << "Updating position for id" << id << std::endl;
		drone_positions[id] = pos;
	      /*}else{
		std::cout << "Adding new position for id" << id << std::endl;
		drone_positions.push_back(pos);
	      }*/
	      //float posmag = sqrt(pos(0)*pos(0) + pos(1)*pos(1) + pos(2)*pos(2));
	      //pos << pos(0)/posmag, pos(1)/posmag, pos(2)/posmag;
	      // Make the vector field u = x, v = y, w = z
	      // Get the corresponding quaternion for the vector
	      Eigen::Quaterniond q;
	      q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1, 0, 0), pos);

	      marker.pose.orientation.x = q.x();
	      marker.pose.orientation.y = q.y();
	      marker.pose.orientation.z = q.z();
	      marker.pose.orientation.w = q.w();
	      marker.id = id + i*1000; 

	      // Sanity check for transform (nan and inf)
	      if (std::isnan(marker.pose.orientation.x) || std::isnan(marker.pose.orientation.y) || std::isnan(marker.pose.orientation.z) || std::isnan(marker.pose.orientation.w)){
		//std::cout << "Nan detected in quaternion" << std::endl;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
	      }
	      if (std::isinf(marker.pose.orientation.x) || std::isinf(marker.pose.orientation.y) || std::isinf(marker.pose.orientation.z) || std::isinf(marker.pose.orientation.w)){
		//std::cout << "Inf detected in quaternion" << std::endl;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
	      }
	      if (std::isnan(marker.pose.position.x) || std::isnan(marker.pose.position.y) || std::isnan(marker.pose.position.z)){
		//std::cout << "Nan detected in position" << std::endl;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
	      }
	      if (std::isinf(marker.pose.position.x) || std::isinf(marker.pose.position.y) || std::isinf(marker.pose.position.z)){
		//std::cout << "Inf detected in position" << std::endl;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
	      }


	      drone_vecs[i].markers.push_back(marker);
	      id++;
	      //}
	    }
	  }
	}
	drone_vec_publishers[i]->publish(drone_vecs[i]);
	std::cout << "Published marker array for drone " << i << std::endl;
      }
    }

  private:
    std::vector<geometry_msgs::msg::PoseStamped> drone_poses;
    std::vector<visualization_msgs::msg::MarkerArray> drone_vecs;
    std::vector<Eigen::Vector3d> drone_positions;
    
    int num_drones; 
    float xextent;
    float yextent;
    float zextent;
    float xdivs;
    float ydivs;
    float zdivs;
    float D;
    float d;
    float num_markers;

    // Interp param
    float c;

    rclcpp::TimerBase::SharedPtr viz_timer; 
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> drone_pose_subscriptions;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> drone_vec_publishers;

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectorFViz>());
  rclcpp::shutdown();
  return 0;
}
