#!/bin/bash

echo "╔══╣ Install: Human Feature Detect (STARTING) ╠══╗"

# Update package list
sudo apt-get update

# Install PCL development libraries
sudo apt-get install -y libpcl-dev

# Install ROS2 dependencies
sudo apt-get install -y \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-rclcpp-lifecycle \
    ros-$ROS_DISTRO-lifecycle-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-std-srvs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs
    
#Source the ROS2 setup file
source /opt/ros/$ROS_DISTRO/setup.bash


echo "╚══╣ Install: Human Feature Detect (FINISHED) ╠══╝"
