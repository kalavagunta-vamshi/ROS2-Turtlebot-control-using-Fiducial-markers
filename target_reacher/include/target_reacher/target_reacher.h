/**
 * @file target_reacher.h
 * @author Darshit Desai (darshit@umd.edu) , Vamshi Kalavagunta (vamshik@umd.edu) , Vinay Bukka(vinay06@umd.edu)
 * @brief This class is used to reach different targets
 * @version 1.0
 * @date 2022-12-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <chrono>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
class TargetReacher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Target Reacher object
     * 
     * @param bot_controller 
     */
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        /**
         * @brief Assignment of bot_controller to m_bot_controller. Declaration of all 
         * parameters from final parms.yaml
         */
        m_bot_controller = bot_controller;
        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");
        this->declare_parameter<std::string>("final_destination.frame_id");

        m_bot_controller->set_goal(aruco_target_x, aruco_target_y); //calling set goal to move the object after retrieving pose from final paramters.
        /**
         * @brief A Subscription object to subscribe goal reached topic and executes callback method when message is published on the topic.
         * Method reached callback to check whether goal reached topic is true.
         */
        goal_subscribe = this->create_subscription<std_msgs::msg::Bool>("/goal_reached", 10,
        std::bind(&TargetReacher::reached_callback, this, std::placeholders::_1));
        /**
         * @brief Publisher node object creation to publish rotation commands for robot once goal reached.
         * 
         */
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);
        /**
         * @brief A Subscription object to subscribe to aruco markers to get the marker id when robot scans
         * after reaching the goal
         * 
         */
        aruco_subscription = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers", 10, std::bind(&TargetReacher::aruco_callback, this, std::placeholders::_1));
        /**
         * @brief A Broadcaster object created to link the transform between final destination and origin frames
         * 
         */
        final_destination_broadcaster =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        /**
         * @brief A Listener Object created to listen to odom topic with a buffer between listening.
         * 
         */
        final_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        final_destination_listener = std::make_shared<tf2_ros::TransformListener>(*final_tf_buffer);
        /**
         * @brief Robot stop counter which will stop the robot after the goal reaches the final goal 
         * 
         */
        goal_stop_count = 1;
        
    }
    

private:
    // attributes
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_subscribe;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> final_destination_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> final_destination_listener;
    std::shared_ptr<BotController> m_bot_controller; //A Shared bot controller object created to interface with bot_controller package.
    std::unique_ptr<tf2_ros::Buffer> final_tf_buffer ;
    int goal_stop_count;
    /**
     * @brief Reached callback method called to check whether the goal reached becomes true and 
     * call rotation callback to publish the rotation.
     * @param msg 
     */
    void reached_callback(const std_msgs::msg::Bool::SharedPtr msg);
    /**
     * @brief A rotation callback is called so that robot rotates to scan the aruco marker
     * 
     */
    void rotation_callback();
    /**
     * @brief Callback method to capture the marker id from aruco marker whenever it is scanned.
     * 
     * @param msg Contains the aruco marker message type, from which marker id is fetched.
     */
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    /**
     * @brief A method to broadcast the link between final_destination and origin  using the pose
     * from final parameters
     * @param x X-coordinate of the specific aruco marker from parameter file
     * @param y Y-coordinate of the specific aruco marker from parameter file
     */
    void final_destination_call(double x,double y);
    /**
     * @brief To get a transform between odom and final destination frames, thereafter robot
     * orients itself to move to final goal.
     */
    void final_listener_callback();
    /**
     * @brief Once the robot has reached final goal, this method has been called to 
     * stop the rotation of the robot by publishing onto cmd_vel topic.
     */
    void final_goal_stop();  
    
};