#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash 
colcon build --symlink-install --packages-up-to aiva_ros_bridge aiva_executor cr_robot_ros2 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='--param ggc-min-expand=20'