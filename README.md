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
---
#### Launch and show it up in rivz2
You can launch rviz2 and show the robot and griiper just type
```
ros2 launch tm_launch tm_rviz_joint_state.launch.py
```
you can see techman robot move in rviz

### Gazebo
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