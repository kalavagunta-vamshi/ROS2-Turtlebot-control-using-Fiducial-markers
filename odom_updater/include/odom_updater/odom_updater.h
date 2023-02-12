/**
 * @file odom_updater.h
 * @author Darshit Desai (darshit@umd.edu) , Vamshi Kalavagunta (vamshik@umd.edu) , Vinay Bukka(vinay06@umd.edu)
 * @brief Frame Publisher Class is used to create a broadcaster between Robot's odom frame
 * and Robot's base_foot_print frame. Subscription is done to robot odom frame so that 
 * the position of robot is broadcasted.
 * @version 1.0
 * @date 2022-12-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

class FramePublisher : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Frame Publisher object
         * 
         */
        FramePublisher() : Node("odom_updater")
        {
            // Using ros declare parameter to declare the robot name
            this->declare_parameter("robot_name", "robot1");
            //Assigning robot name from get parameter to another variable
            group21_robot_name = this->get_parameter("robot_name").as_string();
            /**
             * @brief A Broadcaster node created with the current object
             * 
             */
            odom_broadcaster =  std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            //Using ostream to make the topic related to robot odom frame. This ensures to use different robot names
            std::ostringstream stream;
            stream << "/" << group21_robot_name.c_str() << "/odom";
            std::string topic_name = stream.str();
            /**
             * @brief Creating a subscription object with subscription to the topic name of robot odom and 
             * calls method odom_subscribe_broadcast using bind whenever message is published.
             * 
             */
            odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
                topic_name, 10,
                std::bind(&FramePublisher::odom_subscribe_broadcast, this, std::placeholders::_1));
        }
    private:
        
        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;//Odom Broadcaster definition
        std::string group21_robot_name; // robot name 
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
        void odom_subscribe_broadcast(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
        
        
};