# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_folder = LaunchConfiguration('map_folder')

    map_folder_launch_arg = DeclareLaunchArgument('map_folder', default_value="Maze_ng")

    world_file_name = 'maze.world'
    pkg_dir = get_package_share_directory('robot_spawner_pkg')

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')

    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    tf_helper = Node(package='robot_spawner_pkg', executable='lidar_tf_publisher',
                        output='screen')

    spawn_entity = Node(package='robot_spawner_pkg', executable='spawn_scenario',
                        arguments=['BasicBot', 'en613', map_folder],
                        output='screen')

    return LaunchDescription([
        gazebo,
        tf_helper,
        map_folder_launch_arg,
        spawn_entity,
    ])
