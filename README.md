# Simulation of a Multi-Robot Pick & Place Pipeline for Warehouse Automation
Final Project - Robotics Lab 2025/26
## Available Packages in this Repository ##
- `ros2_iiwa` - ROS2 stack for KUKA iiwa 14 collaborative robots
- `ros2_fra2mo` - containing files to simulate fra2mo (differential drive robot) and the cooperative task
- `aruco_ros` - software package and ROS wrappers of the Aruco Augmented Reality marker detector library
- `m-explore-ros2` - 

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
