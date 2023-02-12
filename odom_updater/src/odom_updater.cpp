#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "odom_updater/odom_updater.h"

/**
 * @brief A method odom_subscribe_broadcast is subscribed using subscription in constructor 
 * and is called continously with the robot pose whenever message is received on this topic.
 * The function broadcasts the position and publishes as the transform from
 * odom to base robot to move the robot.
 * 
 * @param msg The position of the robot that is subscribed.
 */
void FramePublisher::odom_subscribe_broadcast(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    //Creating a message object of type Transformed Stamped to broadcast a connection between robot odom frame
    // robot_base_foot print using the position from odom frame. 
    geometry_msgs::msg::TransformStamped pub;
    pub.header.stamp = this->get_clock()->now();
    pub.header.frame_id = "robot1/odom";
    pub.child_frame_id = "robot1/base_footprint";
    pub.transform.translation.x = msg->pose.pose.position.x;
    pub.transform.translation.y = msg->pose.pose.position.y;
    pub.transform.translation.z = 0.0;
    pub.transform.rotation.x =  msg->pose.pose.orientation.x;
    pub.transform.rotation.y =  msg->pose.pose.orientation.y;
    pub.transform.rotation.z = msg->pose.pose.orientation.z;
    pub.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the Transform message
    odom_broadcaster->sendTransform(pub);
  }

/**
 * @brief Calls the class constructor for odom_updater class and initiates a function call which connects odom and base_footprint frames
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //Creating the object for Frame publisher class using shared pointer and keeping it alive continuously using spin
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}