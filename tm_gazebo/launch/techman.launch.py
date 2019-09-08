import sys
import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def add_gazebo_path(install_dir_path):
    print("this tim install_dir_path is")
    print(install_dir_path)
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir_path + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir_path + "/share"
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir_path + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir_path + '/lib'

def generate_launch_description():
    urdfName = 'robot_hand'
    urdf = os.path.join(get_package_share_directory('tm_grasp_description'), 'urdf/', urdfName + '.urdf')
    print("urdf is ")
    print(urdf)
    #assert os.path.exists(urdf)

    tm_grasp_description_install_dir = get_package_prefix('tm_grasp_description')
    gripper_install_dir = get_package_prefix('robotiq_2f_140_gripper_visualization')
    add_gazebo_path(tm_grasp_description_install_dir)
    add_gazebo_path(gripper_install_dir)
    

    envs = {}
    print("a")
    for key in os.environ.__dict__["_data"]:
        key = key.decode("utf-8")
        if (key.isupper()):
            envs[key] = os.environ[key]
    print("b")
    ld = LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen',
            env=envs
        ),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='techman_robot_utility_scripts', node_executable='spawn_techman.py', arguments=[urdf], output='screen')])
    return ld
