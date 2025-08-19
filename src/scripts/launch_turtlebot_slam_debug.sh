#!/bin/bash
echo "=== TurtleBot3 SLAM Debug Launch ==="

# Terminal 1: Gazebo
echo "Step 1: Starting Gazebo (headless)..."
gnome-terminal --title="1-Gazebo" -- bash -c "
echo 'Starting Gazebo...'; 
source ~/setup_turtlebot.sh; 
ros2 launch gazebo_ros gazebo.launch.py gui:=false; 
exec bash"

sleep 6

# Terminal 2: Spawn Robot  
echo "Step 2: Spawning robot..."
gnome-terminal --title="2-Robot" -- bash -c "
echo 'Spawning TurtleBot3...'; 
source ~/setup_turtlebot.sh; 
ros2 run gazebo_ros spawn_entity.py -file ~/turtlebot_gps_workspace/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -entity turtlebot3 -x 0 -y 0 -z 0.01;
echo 'Robot spawned successfully!';
exec bash"

sleep 4

echo "Step 3: Waiting for scan topic..."
source ~/setup_turtlebot.sh
timeout=30
counter=0
while ! ros2 topic list | grep -q "/scan" && [ $counter -lt $timeout ]; do
    echo "Waiting for /scan topic... ($counter/$timeout)"
    sleep 1
    counter=$((counter + 1))
done

if [ $counter -eq $timeout ]; then
    echo "ERROR: /scan topic never appeared!"
    exit 1
fi

echo "Step 4: Starting SLAM..."
gnome-terminal --title="3-SLAM" -- bash -c "
echo 'Starting SLAM Toolbox...'; 
source ~/setup_turtlebot.sh; 
ros2 launch slam_toolbox online_async_launch.py params_file:=~/turtlebot_gps_workspace/config/slam_params_simple.yaml use_sim_time:=true;
exec bash"

sleep 5

# Terminal 4: GPS Logger
echo "Step 5: Starting GPS Logger..."
gnome-terminal --title="4-GPS" -- bash -c "
echo 'Starting GPS Logger...'; 
source ~/setup_turtlebot.sh; 
python3 auto_gps_logger.py;
exec bash"

echo ""
echo "=== SYSTEM READY ==="
echo "Now run these commands manually:"
echo "1. Open new terminal and run: source ~/setup_turtlebot.sh && rviz2"
echo "2. Open another terminal and run: source ~/setup_turtlebot.sh && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "IMPORTANT: Move robot IMMEDIATELY after starting teleop to initialize SLAM!"

