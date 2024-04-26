# Copyright 2023 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node, PushRosNamespace

pkg_dis_tutorial3 = get_package_share_directory('RINS-task-real-robot')


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),

    DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
                          [pkg_dis_tutorial3, 'maps', 'poligon.yaml']),
                          description='Full path to map yaml file to load'),
]


def generate_launch_description():
    # Directories
    package_dir = get_package_share_directory('RINS-task-real-robot')

    # Paths
    robot_spawn_launch = PathJoinSubstitution(
        [package_dir, 'launch', 'real_turtlebot4_spawn.launch.py'])
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_dis_tutorial3, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_dis_tutorial3, 'launch', 'nav2.launch.py'])

    # Launch configurations
    map_file = LaunchConfiguration('map')


    # Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            # ('namespace', namespace),
            # ('use_sim_time', use_sim_time),
            ('map', map_file),
        ]
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            # ('namespace', namespace),
            # ('use_sim_time', use_sim_time)
        ]
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        # launch_arguments=[
            # ('namespace', namespace),
            # ('use_sim_time', use_sim_time)],
        # condition=IfCondition(LaunchConfiguration('rviz')),
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            # ('namespace', LaunchConfiguration('namespace')),
            # ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )




    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                pkg_dis_tutorial3,
                "config",
                "laser_filter_chain.yaml",
        ])],
        remappings=[
            ('/scan_filtered', 'scan_filtered')
        ]
    )



    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localization)
    ld.add_action(nav2)
    ld.add_action(rviz)

    ld.add_action(laser_filter)

    # ld.add_action(robot_spawn)
    
    return ld
