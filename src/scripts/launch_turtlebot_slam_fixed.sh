#!/bin/bash
echo "Starting TurtleBot3 with SLAM - Proper Order..."

# Terminal 1: Gazebo
echo "Starting Gazebo..."
gnome-terminal --title="Gazebo" -- bash -c "source ~/setup_turtlebot.sh; ros2 launch gazebo_ros gazebo.launch.py gui:=false; exec bash"

sleep 5

# Terminal 2: Spawn Robot
echo "Spawning robot..."
gnome-terminal --title="Spawn Robot" -- bash -c "source ~/setup_turtlebot.sh; ros2 run gazebo_ros spawn_entity.py -file ~/turtlebot_gps_workspace/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -entity turtlebot3 -x 0 -y 0 -z 0.01; exec bash"

sleep 3

# Wait for scan topic to be available
echo "Waiting for robot sensors..."
source ~/setup_turtlebot.sh
while ! ros2 topic list | grep -q "/scan"; do
    echo "Waiting for /scan topic..."
    sleep 1
done

# Terminal 3: SLAM
echo "Starting SLAM..."
gnome-terminal --title="SLAM" -- bash -c "source ~/setup_turtlebot.sh; ros2 launch slam_toolbox online_async_launch.py params_file:=~/turtlebot_gps_workspace/config/slam_params.yaml use_sim_time:=true; exec bash"

sleep 5

# Terminal 4: GPS Logger
gnome-terminal --title="GPS Logger" -- bash -c "source ~/setup_turtlebot.sh; python3 auto_gps_logger.py; exec bash"

echo "System ready! Now manually start RViz and teleop."
echo "Run these commands in new terminals:"
echo "1. rviz2"
echo "2. ros2 run teleop_twist_keyboard teleop_twist_keyboard"
