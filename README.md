This project is a Python and Ubuntu Turtlebot based program that visualizes a Turtlebot in a SLAM world, tracks its movement using GPS data, and reflects the movement onto a Python GUI.

This project also includes a simpler program that utilizes only a Python GUI in which an object is able to move through point-and-click and is able to display objects and backgrounds, with a real world GPS background available. 

Below are directions on how to implement each part of this project:


Video Directions: https://www.youtube.com/watch?v=wjsC5dGPH0w




Simple Python GUI ----------------------------------------------------------------------------------------------------------------------------------

How to use:

Step 1: Open the `gui_receiver` file
Step 2: Run the `animating_sender` file in the terminal
Step 3: Choose your presets
Step 4 (optional): If you run the GPS background option, include the coordinates of what area youd like visualized as a background on the GUI, as well as the level of zoom you would like to have on the GPS background. (UIC Coordinates: 41.8708, -87.6505)
Step 5: Apply your presets

Once both scripts are running, you can now click on the interface. The object will smoothly move to the point you clicked, continuing to move as you click more. You can also change the speed and size of the object using the controls on the right. Clicking the "Center Object" option will bring the 


What each script does:

- gui_receiver.py launches a visual interface where:
- You can click anywhere to send a target location to the sender.
- Users can choose preset backgrounds and characters using dropdowns.
- There is a live slider for object size.
- A dropdown lets you select animation speed: slow, normal, or fast.
- A “Center Character” button resets the object to the center of the canvas.
- Starts a background thread to listen for live position updates from the sender.
- When it receives position data, it moves the object smoothly using imshow.
- animating_sender.py is a background script that:
- Connects to the GUI over a local socket (127.0.0.1).
- Listens for coordinate messages like "x,y,speed".
- Animates the object by sending incremental updates from its current position to the new target.
- Adjusts speed by changing how many steps and how fast each one is sent:
- Slow = 70 steps with delay
- Normal = 50 steps
- Fast = 30 steps with shorter delay

How the scripts communicate:

- The two scripts use Python sockets over localhost:
- The sender connects first and waits.
- When the GUI launches, it accepts the sender's connection.
- When the user clicks, the GUI sends a message like "40.3,60.7,fast" to the sender.
- The sender processes that and sends back a series of intermediate positions.
- The GUI listens for those and animates the object’s movement accordingly.

Ubuntu Turtlebot + GPS ----------------------------------------------------------------------------------------------------------------------------------

Run Ubuntu TurtleBot + GPS:

Uses:

- Ubuntu 22.04 VM through VirtualBox
- ROS2
- RViz
- TurtleBot3\
- VS Editor

Use this RViz display configuration:

- Fixed Frame: "odom"
- Add Robot Model
- Add LaserScan (topic: /scan)
- Add TF

Step 1 - 

Run the launch file:

./launch_turtlebot_house_with_gps.sh


Step 2 - 

Run the teleop control in a separate terminal, move with "uiojklm,." keys as explained in terminal:

source ~/setup_turtlebot.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard


Step 3:

Run the Python GUI file `gui_receiver` and select your background and object


Step 4:

In the powershell terminal, run the bridge file `windows_gps_bridge` to bridge the Turtlebot to the Python GUI:

python "C:\Users\georg\python_projects_all\matplotlibPROJECT\matplotlib_project\gui_coord_sender\windows_gps_bridge.py"



Step 5:

After proper connection between the Turtlebot, bridge and GUI, movement of the Turtlebot in your Ubuntu VM on RViz should be reflected onto your Python GUI





