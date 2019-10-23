## Techman robot Ros2 and grasp
This is a ros2 example by using techman robot and robotiq.
#### launch and show it up in rivz2
You can launch rviz2 and show the robot and griiper just type
```
ros2 launch tm_launch tm_rviz_joint_state.launch.py
```
you can see techman robot move in rviz

#### launch and show it up in Gazebo
You can launch Gazebo and show the robot and griiper just type
```
ros2 launch tm_launch tm_gazebo_move.launch.py
```
you can see techman robot stand in gazebo environment

#### send command to Gazebo
You can send command to gazebo by typing
```
run tm_driver send_tm_command
```
you can see techman robot run in gazebo environment

#### get eye in hand camera image in Gazebo

If you want to see the image from eye in hand camera, you can type 
```
ros2 run tm_driver tm_get_img
```
you can see the image from the eye in hand camera

