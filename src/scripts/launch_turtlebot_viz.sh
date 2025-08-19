#!/bin/bash
echo "Starting TurtleBot3 with RViz Visualization..."
echo "This will open 4 terminals automatically"

# Terminal 1: Gazebo
gnome-terminal -- bash -c "source ~/setup_turtlebot.sh; ros2 launch gazebo_ros gazebo.launch.py gui:=false; exec bash"

sleep 3

# Terminal 2: Spawn Robot
gnome-terminal -- bash -c "source ~/setup_turtlebot.sh; ros2 run gazebo_ros spawn_entity.py -file ~/turtlebot_gps_workspace/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -entity turtlebot3 -x 0 -y 0 -z 0.01; exec bash"

sleep 2

# Terminal 3: RViz
gnome-terminal -- bash -c "source ~/setup_turtlebot.sh; rviz2 -d ~/turtlebot_config.rviz; exec bash"

# Terminal 4: GPS Logger
gnome-terminal -- bash -c "source ~/setup_turtlebot.sh; python3 auto_gps_logger.py; exec bash"

echo "All terminals launched!"
