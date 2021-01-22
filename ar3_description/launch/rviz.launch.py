# Author: Kevin DeMarco

import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    #rviz_config = os.path.join(get_package_share_directory('ar3_description'),
    #                          'rviz', 'ar3.rviz')

    xacro_path = os.path.join(get_package_share_directory('ar3_description'),
                              'urdf', 'ar3.urdf.xacro')
    robot_description = {'robot_description' : Command(['xacro', ' ', xacro_path])}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        robot_description],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            #arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time},
                        robot_description],
            output='screen'),
    ])
