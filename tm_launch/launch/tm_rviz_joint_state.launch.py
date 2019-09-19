import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    urdfName = 'robot_hand'
    urdf = os.path.join(get_package_share_directory('tm_grasp_description'), 'urdf/', urdfName + '.urdf')
    rvizPath = os.path.join(get_package_share_directory('tm_grasp_description'), 'rviz/', urdfName + '.rviz')
    rviz_config_dir = os.path.join(
           urdf,
            'rviz',
            rvizPath)
    return LaunchDescription([
        Node(package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='screen', arguments=[urdf]),
        Node(package='tm_status', node_executable='send_joint_command', output='screen')
    ])
