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
3. Build the Package
After creating the package, build it using catkin_make.
cd ~/catkin_ws
catkin_make
4. Install TurtleBot3 Simulation Packages
You will need to install the TurtleBot3 simulation packages to simulate the robot.
sudo apt-get install ros-noetic-turtlebot3-simulations
5. Modify .bashrc
Next, open the .bashrc file to set up environment variables.
nano ~/.bashrc
Add the following lines to the end of the file:
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
Save the file and exit by pressing Ctrl+X, then Y, and Enter.

To apply the changes, run:
source ~/.bashrc
