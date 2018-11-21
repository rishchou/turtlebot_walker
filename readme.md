# Turtle-bot walker
<a href='https://github.com/rishchou/turtlebot_walker/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>

## Project overview
A simple turtlebot walker algorithm implementation similar to roomba vacuum cleaner robot.

## Dependencies

This ROS node is made to be used on systems which have:

    ROS Kinetic
    Ubuntu 16.04
    Turtlebot packages

To install ROS, follow the instructions on this link: http://wiki.ros.org/kinetic/Installation

To install the turtlebot packages, run the following after installing ROS Kinetic on your ubuntu 16.04.

sudo apt-get install ros-kinetic-turtlebot-*

This installs all the turtlebot packages.

##  Build

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/rishchou/turtlebot_walker.git
cd ..
catkin_make
```
## Run demo

After following the installation instructions above, you can either run it using roslaunch by typing the following command in the terminal. It will start both the turtlebot gazebo simulation and the walker node in a separate terminal.
```
roslaunch turtlebot_walker turtlebot_walker.launch
```
If you'd like to run the nodes separately, then run roscore in the terminal as given below
```
roscore
```
We need to launch the turtlebot simulation. Run the command below in a new terminal.
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
Then run the walker ros node by running the command below in a new terminal.
```
rosrun turtlebot_walker turtlebot_walker
```
## Recording bag files with the launch file

You must first build the project using catkin_make as described earlier. You may run the command below to launch the nodes and record all the topics except the camera data. The bag file will be in the results directory once the recording is complete. By default it records for 30 seconds and you can change it in the optional argument secs
```
roslaunch turtlebot_walker turtlebot_walker.launch record:=true record_time:=32
```
The bag file is stored in ..turtlebot_walker/results/turtlebot_walker.bag
## Playing back the bag file

First, Go to the results folder.

cd <path to repository>/results

To inspect the bag file, ensure that the roscore is running. Then in a new terminal, enter the command below while in the results directory.
```
rosbag play turtlebot_walker.bag
```
You will be able to see the elapsed time output on the screen. It would be playing the recorded messages when the turtlebot was actually moving.

You can view the messages being published on the topic /cmd_vel_mux/input/navi.
```
rostopic echo /cmd_vel_mux/input/navi
```
