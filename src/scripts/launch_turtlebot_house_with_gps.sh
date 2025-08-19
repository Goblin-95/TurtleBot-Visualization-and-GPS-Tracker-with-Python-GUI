#!/bin/bash

echo "========================================================"
echo "🐢 TurtleBot3 House World with DYNAMIC GPS System 🗺️"
echo "========================================================"
echo ""

# Check if setup script exists
if [ ! -f ~/setup_turtlebot.sh ]; then
    echo "❌ Error: ~/setup_turtlebot.sh not found!"
    echo "Please make sure your setup script exists."
    exit 1
fi

echo "🏠 Starting TurtleBot3 House World..."
# Terminal 1: House World (Gazebo)
gnome-terminal --title="1-Gazebo-House" -- bash -c "
echo '🏠 Starting Gazebo House World...';
source ~/setup_turtlebot.sh;
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py;
exec bash"

echo "⏳ Waiting for Gazebo to load..."
sleep 8

echo "📡 Starting Dynamic GPS Publisher..."
# Terminal 2: Dynamic GPS Publisher
gnome-terminal --title="2-Dynamic-GPS" -- bash -c "
echo '📡 Starting Dynamic GPS Publisher...';
echo 'Converting robot movement to GPS coordinates';
source ~/setup_turtlebot.sh;
python3 ~/turtlebot_gps_workspace/dynamic_gps_publisher.py;
exec bash"

sleep 3

echo "🗺️ Starting SLAM..."
# Terminal 3: SLAM
gnome-terminal --title="3-SLAM" -- bash -c "
echo '🗺️ Starting SLAM Toolbox...';
source ~/setup_turtlebot.sh;
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true;
exec bash"

sleep 2

echo "📝 Starting GPS Logger..."
# Terminal 4: GPS Logger
gnome-terminal --title="4-GPS-Logger" -- bash -c "
echo '📝 Starting GPS Data Logger...';
echo 'Logging GPS data to JSON file for Python GUI';
source ~/setup_turtlebot.sh;
python3 ~/turtlebot_gps_workspace/auto_gps_logger.py;
exec bash"

sleep 2

echo "👁️ Starting RViz Visualization..."
# Terminal 5: RViz (automatically opens)
gnome-terminal --title="5-RViz" -- bash -c "
echo '👁️ Starting RViz for robot visualization...';
echo 'RViz will show robot, laser scan, and SLAM map';
source ~/setup_turtlebot.sh;
rviz2;
exec bash"

sleep 3

echo "🎮 Starting Robot Teleop Control..."
# Terminal 6: Working Teleop (the one that actually works)
gnome-terminal --title="6-Robot-Control" -- bash -c "
echo '🎮 ROBOT TELEOP CONTROLS';
echo '========================';
echo 'Use these keys to move the robot:';
echo '   u    i    o   ';
echo '   j    k    l   ';
echo '   m    ,    .   ';
echo '';
echo 'i = move forward    , = move backward';
echo 'j = turn left       l = turn right';
echo 'u = forward+left    o = forward+right';
echo 'm = backward+left   . = backward+right';
echo 'k = stop            CTRL-C = quit';
echo '';
echo 'SPACE or k = force stop';
echo 'q/z = increase/decrease max speeds';
echo 'w/x = increase/decrease linear speed';
echo 'e/c = increase/decrease angular speed';
echo '';
echo '📍 Watch GPS coordinates change in Terminal 4!';
echo '🗺️ Watch robot move in RViz (Terminal 5)!';
echo '';
source ~/setup_turtlebot.sh;
ros2 run teleop_twist_keyboard teleop_twist_keyboard;
exec bash"

echo ""
echo "✅ SYSTEM STARTUP COMPLETE!"
echo "========================================================"
echo "📋 TERMINALS OPENED:"
echo "  1️⃣  Gazebo House World (simulation environment)"
echo "  2️⃣  Dynamic GPS Publisher (converts movement → GPS)"
echo "  3️⃣  SLAM (mapping and localization)"
echo "  4️⃣  GPS Logger (saves data for Python GUI)"
echo "  5️⃣  RViz (robot visualization)"
echo "  6️⃣  Robot Teleop Control (move the robot!)"
echo ""
echo "🚀 HOW TO USE:"
echo "1. Wait for all terminals to fully load (~30 seconds)"
echo "2. Focus on Terminal 6 (Robot Control) to move robot"
echo "3. Watch GPS coordinates change in Terminal 4"
echo "4. Watch robot move in RViz (Terminal 5)"
echo "5. Run your Python GUI separately to see live GPS data"
echo ""
echo "🔧 TROUBLESHOOTING:"
echo "- If Gazebo is slow to load, wait longer before using teleop"
echo "- If GPS data isn't changing, check Terminal 2 for errors"
echo "- If RViz is empty, make sure Fixed Frame is set to 'odom' or 'map'"
echo ""
echo "Press CTRL+C in any terminal to stop that component"
echo "========================================================"
