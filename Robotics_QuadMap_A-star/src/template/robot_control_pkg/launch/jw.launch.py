import os

from launch import LaunchDescription
from launch_ros.actions import Node

#Addt'l from Midterm
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    return LaunchDescription([
        Node(package='robot_control_pkg', executable='jw_control',
         output='screen'),
        Node(package='robot_control_pkg', executable='jw_mapper',
        	 output='screen'),
        Node(package='robot_control_pkg', executable='jw_planner', 
            output='screen'),
    ])

