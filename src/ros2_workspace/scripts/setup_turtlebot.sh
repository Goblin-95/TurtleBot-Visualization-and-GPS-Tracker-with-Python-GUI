#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot_gps_workspace
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot_gps_workspace/turtlebot3_gazebo/models

