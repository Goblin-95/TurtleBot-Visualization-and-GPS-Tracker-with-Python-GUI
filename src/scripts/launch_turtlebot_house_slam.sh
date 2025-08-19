#!/bin/bash
echo "=== TurtleBot3 SLAM with House World ==="

# Terminal 1: TurtleBot3 House World (includes robot automatically)
echo "Starting TurtleBot3 House World..."
gnome-terminal --title="1-House-World" -- bash -c "
echo 'Starting TurtleBot3 House World...'; 
source ~/setup_turtlebot.sh; 
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py || echo 'House launch failed, trying manual approach...';
exec bash"

sleep 10

# Terminal 2: SLAM
echo "Starting SLAM..."
gnome-terminal --title="2-SLAM" -- bash -c "
echo 'Starting SLAM Toolbox...'; 
source ~/setup_turtlebot.sh; 
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true;
exec bash"

sleep 3

# Terminal 3: GPS Logger  
echo "Starting GPS Logger..."
gnome-terminal --title="3-GPS" -- bash -c "
echo 'Starting GPS Logger...'; 
source ~/setup_turtlebot.sh; 
python3 auto_gps_logger.py;
exec bash"

echo ""
echo "=== SYSTEM READY ==="
echo "The robot is now in a house with walls, furniture, and obstacles!"
echo "Start RViz and teleop to begin mapping."
echo ""
echo "Commands to run in new terminals:"
echo "1. source ~/setup_turtlebot.sh && rviz2"  
echo "2. source ~/setup_turtlebot.sh && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
