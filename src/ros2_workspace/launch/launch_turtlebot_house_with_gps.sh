#!/bin/bash

echo "=== TurtleBot3 House World with DYNAMIC GPS ==="

# Terminal 1: House World
gnome-terminal --title="1-House" -- bash -c "
source ~/setup_turtlebot.sh;
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py;
exec bash"

sleep 8

# Terminal 2: Dynamic GPS Publisher (converts robot movement to GPS)
gnome-terminal --title="2-Dynamic-GPS" -- bash -c "
source ~/setup_turtlebot.sh;
echo 'Starting Dynamic GPS Publisher...';
echo 'This converts robot movement to changing GPS coordinates';
python3 ~/turtlebot_gps_workspace/dynamic_gps_publisher.py;
exec bash"

sleep 3

# Terminal 3: SLAM
gnome-terminal --title="3-SLAM" -- bash -c "
source ~/setup_turtlebot.sh;
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true;
exec bash"

sleep 2

# Terminal 4: GPS Logger
gnome-terminal --title="4-GPS-Logger" -- bash -c "
source ~/setup_turtlebot.sh;
python3 ~/turtlebot_gps_workspace/auto_gps_logger.py;
exec bash"

# Terminal 5: Robot Control with custom keys
sleep 2
gnome-terminal --title="5-Custom-Teleop" -- bash -c "
source ~/setup_turtlebot.sh;
echo '🎮 Custom Robot Controls:';
echo 'Use uiojklm,. keys to move robot';
echo 'i=forward, ,=backward, j/l=turn, k=stop';
echo 'GPS coordinates will change as robot moves!';
python3 ~/turtlebot_gps_workspace/custom_teleop.py;
exec bash"

echo ""
echo "🚀 System ready with DYNAMIC GPS!"
echo "📍 GPS coordinates will now change as robot moves!"
echo "🎮 Use Terminal 5 (Custom Teleop) to move robot with uiojklm,. keys"
echo "📊 Watch GPS data in Terminal 4 - coordinates should change!"
