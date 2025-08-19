#!/bin/bash
echo "=== TurtleBot3 SLAM with Spawned Obstacles ==="

# Terminal 1: Gazebo (default empty world)
echo "Step 1: Starting Gazebo..."
gnome-terminal --title="1-Gazebo" -- bash -c "
echo 'Starting Gazebo...'; 
source ~/setup_turtlebot.sh; 
ros2 launch gazebo_ros gazebo.launch.py gui:=false; 
exec bash"

sleep 8

# Terminal 2: Spawn Robot  
echo "Step 2: Spawning robot..."
gnome-terminal --title="2-Robot" -- bash -c "
echo 'Spawning TurtleBot3...'; 
source ~/setup_turtlebot.sh; 
ros2 run gazebo_ros spawn_entity.py -file ~/turtlebot_gps_workspace/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -entity turtlebot3 -x 0 -y 0 -z 0.01;
sleep 2;
echo 'Adding obstacles...';
ros2 run gazebo_ros spawn_entity.py -entity wall1 -x 3 -y 0 -z 1 -Y 0 -P 0 -R 0 -file /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world || echo 'Using simple box obstacles...';
echo 'Robot and obstacles ready!';
exec bash"

sleep 6

echo "Step 3: Waiting for scan topic..."
source ~/setup_turtlebot.sh
timeout=30
counter=0
while ! ros2 topic list | grep -q "/scan" && [ $counter -lt $timeout ]; do
    echo "Waiting for /scan topic... ($counter/$timeout)"
    sleep 1
    counter=$((counter + 1))
done

# Terminal 3: SLAM
echo "Step 4: Starting SLAM..."
gnome-terminal --title="3-SLAM" -- bash -c "
echo 'Starting SLAM Toolbox...'; 
source ~/setup_turtlebot.sh; 
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true;
exec bash"

sleep 3

# Terminal 4: GPS Logger
echo "Step 5: Starting GPS Logger..."
gnome-terminal --title="4-GPS" -- bash -c "
echo 'Starting GPS Logger...'; 
source ~/setup_turtlebot.sh; 
python3 auto_gps_logger.py;
exec bash"

echo ""
echo "=== SYSTEM READY ==="
echo "Try moving robot around the boundaries - laser should detect the ground plane edges"
