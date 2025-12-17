# RL2025HW01
Bring up your robot
## Available Packages in this Repository ##
- `armando_description` - robot description and configuration files
- `armando_gazebo` - launch and run-time configurations
- `armando_controller` - implementation of dedicated controllers

## Getting Started

```shell
cd ~/ros2_ws
git clone https://github.com/gianmarco-ferrara/RL2025HW01.git
colcon build 
source install/setup.bash
```
# **Usage**
 ## **1. Launch the Manipulator in Rviz**

In order to start the manipulator simulation in Rviz, open your first terminal and run:
```shell
ros2 launch armando_description armando_display.launch.py 
```
rviz will be started. The collision meshes can be seen enabling the collision detection (they have been substituted with boxes).

 ## **2. Launch the Manipulator in Gazebo**
In order to start the manipulator simulation in Gazebo run:
```shell
ros2 launch armando_gazebo armando_world.launch.py
```
The manipulator will be in a pre-defined position.
## **3. Camera Sensor**
After launching the manipulator in Gazebo in another terminal type:
```shell
ros2 run rqt_image_view rqt_image_view
```
## **4. Start Controller**
Once gazebo is opened, in another terminal it is possible to start a controller.
Two controllers are available:
- Position Controller
- Trajectory Controller
They can be executed with the command:
```shell
ros2 launch armando_controller armando_control.launch.py controller_type:=<type>
```
where `<type>` can be `position` or `trajectory`.
It is possible to list all the running controllers with:
```shell
ros2 control list_controllers
```
Before activating another controller it is possible to deactivate the running one:
```shell
ros2 control switch_controllers --stop <controller_name>
```
## **5. Subscriber and Publisher node**
It is possible to launch `arm_controller_node` in order to see the robot following four desired positions specified in the code.
```shell
ros2 run armando_controller arm_controller_node --ros-args -p use_trajectory:=<value>
```
where `<value>` is set to `false` to use the position controller or `true` to use the trajectory controller.
# TEMP-TechnicalProject
# TEMP-TechnicalProject
