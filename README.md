# ROS2-Turtlebot-control-using-Fiducial-markers
A final project for the Robotics Programming course where we used ROS2 Galactic to control turtlebot's motion and guide it to it's target location given by fiducial markers

To run the ROS project successfully there are couple of steps:

-Create a workspace ~/ros2_ws/src

-In the src folder clone the following project using command git clone https://github.com/zeidk/enpm809y_FinalFall2022.git

-In your .bashrc add the following commands:

  – export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path>/<to>/tb3_gazebo/models
  
  – alias frames='cd /tmp && ros2 run tf2_tools view_frames && evince frames.pdf'

-Now since this package already contains target_reacher package replace the package by cloning the package developed from this repository along with the odom_updater package.

Tasks for this project:

1. The first task is to create a broadcaster which connects the robots odom with the world odom. This is done using the odom_updater package

2. The second task would be to move the robot to the goal coordinates using the aruco markers and target_reacher package.

The final output of the project looks like this:

The connected tree as shown below using the package odom_updater which broadcasts robot's odom to world frame continuously:


