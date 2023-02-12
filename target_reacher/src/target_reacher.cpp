#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include<string>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
using namespace std::chrono_literals;

void TargetReacher::reached_callback(const std_msgs::msg::Bool::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Goal Reached: '%d'", msg->data);
   
    if(msg->data ==1 ){
       rotation_callback();
       
    }
}

void TargetReacher::rotation_callback()
{    if(goal_stop_count==1){
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0.0;
      message.angular.z = 0.2;
      cmd_vel_publisher->publish(message); //publishing the twist message for robot to rotate
        }
    else if(goal_stop_count==0){
        final_goal_stop();
    }

}

void TargetReacher::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    int marker_id = msg->marker_ids.at(0);
    std::string marker_id_string = std::to_string(marker_id);
    std::string final_x;
    std::string final_y;
    final_x.append("final_destination.aruco_");
    final_y.append("final_destination.aruco_");
    final_x.append(marker_id_string);
    final_y.append(marker_id_string);
    final_x.append(".x"); //Making of x parameter for the specific aruco marker to be retrieved from parameter file.
    final_y.append(".y"); //Making of y parameter for the specific aruco marker to be retrieved from parameter file.
    auto final_destination_x = this->get_parameter(final_x).as_double(); //getting the x coordinate from parameter file.
    auto final_destination_y = this->get_parameter(final_y).as_double(); //getting the y coordinate from parameter file.
    final_destination_call(final_destination_x,final_destination_y);
    


}
void TargetReacher::final_destination_call(double x,double y){
    geometry_msgs::msg::TransformStamped final_pub;
    final_pub.header.stamp = this->get_clock()->now();
    final_pub.header.frame_id = this->get_parameter("final_destination.frame_id").as_string(); //Frame Id can be changed in final parameters to go to a different goal point marker.
    final_pub.child_frame_id = "final_destination";

    final_pub.transform.translation.x = x;
    final_pub.transform.translation.y = y;
    final_pub.transform.translation.z = 0.0;

    final_pub.transform.rotation.x = 0;
    final_pub.transform.rotation.y = 0;
    final_pub.transform.rotation.z = 0;
    final_pub.transform.rotation.w = 1;
    for (int i=0 ;i<100 ;i++)
    {
        final_destination_broadcaster->sendTransform(final_pub); //broadcast transformation between origin and final destination 
    }
    final_listener_callback();
}

void TargetReacher::final_listener_callback(){
    geometry_msgs::msg::TransformStamped final_sub;
    final_sub = final_tf_buffer->lookupTransform("robot1/odom", "final_destination", tf2::TimePointZero);
    auto goal2_x = final_sub.transform.translation.x;
    auto goal2_y = final_sub.transform.translation.y;
    m_bot_controller->set_goal(goal2_x, goal2_y); //Asking the robot to go to goal2 or final goal
    goal_stop_count=0;
    
    

}
void TargetReacher::final_goal_stop(){
    auto message = geometry_msgs::msg::Twist();
    message.angular.z = 0.0;
    cmd_vel_publisher->publish(message); //publishing the twist message for robot to stop rotation
}


