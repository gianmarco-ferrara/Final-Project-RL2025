# Simulation of a Multi-Robot Pick & Place Pipeline for Warehouse Automation
Final Project - Robotics Lab 2025/26

## General Overview
The actors of the scene are fra2mo (differential drive robot home-made by Prisma Lab @ UNINA) and two Iiwa robots (by Kuka) in a custom warehouse setting. Each manipulator manages a workstation, while the mobile robot is required to transfer an object from the loading workstation to the unloading one. The NAV 2 stack is exploited to plan long-range autonomous navigation missions within the warehouse. Then, a visual servoing logic (using ArUco tags) is used for final precise docking. Coordination with manipulators is guaranteed by means of an handshaking logic (implemented with some custom coordination topics). Vacuum grippers are simulated via Detachable Joint plugin.
## Available Packages in this Repository ##
- `ros2_iiwa` - ROS2 stack for KUKA iiwa 14 collaborative robots
- `ros2_fra2mo` - containing files to simulate fra2mo (differential drive robot) and the cooperative task
- `aruco_ros` - software package and ROS wrappers of the Aruco Augmented Reality marker detector library
- `m-explore-ros2` - for greedy frontier-based exploration

## Getting Started
```shell
cd ~/ros2_ws
git clone https://github.com/gianmarco-ferrara/Final-Project-RL2025.git
colcon build 
source install/setup.bash
```
# **Usage**
In order to start the simulation of the cooperative task, open a terminal and execute the command:

```shell
ros2 launch ros2_fra2mo cooperative_task.launch.py
```
This command allows you to visualize the execution of the task in Gazebo and to monitor fra2mo in RViz.

If you want to record fra2mo's trajectory during task execution, open a second terminal and run (before fra2mo starts moving):
```shell
ros2 run ros2_fra2mo plot_trajectory.py
```
at the end of the task execution, press `ctrl+C` to get the image in .png format.
### Demo
A video-demo is available here:
[![Guarda il video](https://img.youtube.com/vi/6dcHrBN-tpY/maxresdefault.jpg)](https://www.youtube.com/watch?v=6dcHrBN-tpY)

## Other available functionalities
### Visualization of the environment and spawning of the robots
In order to spawn the robots in Gazebo (and start nodes related to vision and navigation) without starting the execution of the task, open a terminal and use the command:
```shell
ros2 launch ros2_fra2mo fra2mo_iiwa.launch.py
```
### Exploration
In a preliminary phase, exploration of the unknown environment has been used to obtain an occupancy map. To reproduce the exploration, open a terminal and use the command:
```shell
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
```
then in another terminal launch the exploration:
```shell
ros2 launch ros2_fra2mo fra2mo_exploration.launch.py
```
At the end of the exploration procedure, open the `ros2_fra2mo/maps` folder in a third terminal and use the command:
```shell
ros2 run nav2_map_server map_saver_cli -f map
```
### Autonomous Navigation (AMCL)
Open a terminal and spawn fra2mo in Gazebo using the command:
```shell
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
```
then in another terminal start navigation by using:
```shell
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```
Now you can assign Nav2 goal poses directly in RViz and see fra2mo autonomously navigating the environment. 
### RViz GUI to plan manipulators' trajectory
Make sure `joint_state_publisher_gui` is available within your workspace. If not, supposing you are using ROS 2 Humble, you can install it directly from the terminal:
```shell
sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui
```
Then you can visualize iiwa1 in RViz by launching:
```shell
ros2 launch ros2_fra2mo teaching_iiwa.launch.py
```
or iiwa2 by:
```shell
ros2 launch ros2_fra2mo teaching_sec_iiwa.launch.py
```
It is suggested to activate the TF Display in order to visualize also the target frames used for planning.
