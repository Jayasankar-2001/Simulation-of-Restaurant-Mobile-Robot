# Simulation-of-Restaurant-Mobile-Robot
This project was given by Goat Robotics as part of their ROS Developer job role. The task involves setting up a ROS environment, creating a package, and simulating a TurtleBot3 robot to handle multiple orders in a café environment. The specific tasks provided are outlined below.
##Installation and Setup Instructions
#1. Install ROS Noetic on Ubuntu 20.04
To begin, you need to install ROS Noetic on your Ubuntu 20.04 system. You can follow the official ROS installation guide [here](http://wiki.ros.org/noetic/Installation/Ubuntu).
### 2. Create a ROS Package
Once ROS is installed, create a new ROS package in your workspace.

```bash
cd ~/catkin_ws/src
catkin_create_pkg goat_cafe std_msgs rospy roscpp 
```
3. Build the Package
After creating the package, build it using catkin_make.
```bash
cd ~/catkin_ws
catkin_make
```
5. Install TurtleBot3 Simulation Packages
You will need to install the TurtleBot3 simulation packages to simulate the robot.
```bash
sudo apt-get install ros-noetic-turtlebot3-simulations
```
7. Modify .bashrc
Next, open the .bashrc file to set up environment variables.
```bash
nano ~/.bashrc
```
Add the following lines to the end of the file:
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
```
Save the file and exit by pressing Ctrl+X, then Y, and Enter.

To apply the changes, run:
```bash
source ~/.bashrc
```
6. Launch the ROS Environment
Launch the Bringup
In the first terminal, launch the bringup file:
```bash
roslaunch goat_cafe bringup.launch
```
Launch the Navigation
In a second terminal, launch the navigation:
```bash
roslaunch goat_cafe navigation.launch
```
Run the Café Simulation Script
In a third terminal, run the café simulation script:
```bash
rosrun goat_cafe cafe_v0.01.py
```
7. GUI for Taking Orders
To handle orders from different tables, use the following command with the appropriate table number:
```bash
rosrun goat_cafe table_gui.py _table_number:=<table_number>
```
For example, to take an order from table 1:
```bash
rosrun goat_cafe table_gui.py _table_number:=1
```
