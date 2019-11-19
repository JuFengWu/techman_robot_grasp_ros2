## Techman robot Ros2 and grasp
This is a ros2 example by using techman robot and robotiq.

### Install 
---
#### Create a ROS 2.0 workspace
Create the workspace and download source files
```
mkdir -p ~/jufeng_tm_ws/src
git clone https://github.com/JuFengWu/techman_robot_grasp_ros2.git
```
Build this project and source it
```
source /opt/ros/dashing/setup.bash
cd ~/jufeng_tm_ws
colcon build
source install/setup.bash
```


### Rviz

Rviz is a powerful robot simulator in Ros. Current it have ros2 version and sometimes it is called Rviz2.

The Rviz in Ros2 is almost the same as Ros1, so if you used Rviz in Ros1 before, it is easy to get start!

---
#### Launch and show it up in rivz2
You can launch rviz2 and show the robot and griiper just type
```
ros2 launch tm_launch tm_rviz_joint_state.launch.py
```
you can see techman robot move in rviz

### Gazebo

Gazebo is a powerful simulator with high-performance physics engines and because its physics engines it is a good data creator. 
The difference between Gazebo and Rviz is that Gazebo is a robot status data creator and Rviz is a data receiver to show up the robot status.

---
#### Launch and show it up in Gazebo
You can launch Gazebo and show the robot and griiper just type
```
ros2 launch tm_launch tm_gazebo_move.launch.py
```
you can see techman robot stand in gazebo environment

#### Send command to Gazebo
You can send command to gazebo by typing
```
ros2 run tm_driver send_tm_command
```
you can see techman robot run in gazebo environment

#### Get eye in hand camera image in Gazebo

If you want to see the image from eye in hand camera, you can type 
```
ros2 run tm_driver tm_get_img
```
you can see the image from the eye in hand camera

### Moveit 

Moveit is a power path planning tool, but currently the Ros2 have only alpha version.

:warning:

1. Currently moveit recommand used in Ros1 envirnoment, so this demo use Ros1 and Ros2 bridge to connect Ros1 and Ros2. Make sure your computer have installed Ros1 Melodic.
2. This test is not only use this project, but also use [this project](https://github.com/JuFengWu/tm_robot_ros2_ros1_bridge).
---
#### Send a target to moveit and get the trajectory back.

1. Download [this package](https://github.com/JuFengWu/tm_robot_ros2_ros1_bridge) and follow this instruction to build and install this project.
2. Open a terminal and type ``roslaunch moveit_action_pkg tm_movit_bringup.launch `` 
3. Open a new terminal to create ros2 ros1_bridge by typing those cmds
```
. /opt/ros/melodic/setup.bash
. /opt/ros/dashing/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge __log_disable_rosout:=true
```
4. Open a new terminal and type ``ros2 run tm_status get_trajectory``
5. You can see the third terminal show up the trajecoty which is calculated from the first terminal.