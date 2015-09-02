# khan_robot

Basic ROS description for the Kinematic Helper and Autonomous Navigator (K.H.A.N.), a low-cost ROS-enabled mobile robot designed for educational purposes.

# Installation Instructions

## Prerequisites

This package has been tested and confirmed with the following basic prerequisites:

1. [Ubuntu](http://www.ubuntu.com) 14.04
2. [ROS](http://www.ros.org) Indigo
3. [Gazebo](http://gazebosim.org/) 4
4. A working/configured catkin workspace.

**Note** ROS Indigo and Gazebo 4 is not a standard configuration, but is the way DRC related packages are configured, hence the holdover. You can skip the Gazebo Integration step below if you are using Gazebo 2 (this setup hasn't been tested.)

## Installing Ubuntu Packages

In order to install this package, you need to clone this repository into your catkin workspace src directory. You also need to install the following packages, typically using the Ubuntu Package Management System:

1. ros-indigo-driver-base
2. ros-indigo-polled-camera
3. ros-indigo-camera-info-manager
4. ros-indigo-ros-control
5. ros-indigo-twist-mux
6. ros-indigo-diff-drive-controller
7. ros-indigo-joint-state-controller

## Gazebo 4 Integration

Due to using ROS Indigo and Gazebo 4, there are a few bugs in the Gazebo/ROS Control interfaces. In order to fix these, the [gazebo-ros-pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) must be cloned into your catkin workspace and built from source on the jade-devel branch.

## Building

From the base of your catkin workspace, execute `catkin_make`

# Launching the Simulation

Once you have the code built, you can run K.H.A.N. in Gazebo by executing `roslaunch khan_launch khan_gazebo.launch`

# Driving K.H.A.N.

In Gazebo, you can drive K.H.A.N. by sending a [Twist](https://en.wikipedia.org/wiki/Screw_theory#Twist) [message](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html), which indicates the velocities the robot should attempt to move. These messages are received on the standard `/cmd_vel` topic, as can be seen in the relevant ROS [tutorial](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics).
