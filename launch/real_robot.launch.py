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

# iz turtlebot4_spawn.launch.py, ker prej se je od tam zagnal rviz
# launch.actions pa ne bom dodal, ker pomoje na rabimo
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace

pkg_dis_tutorial3 = get_package_share_directory('RINS-task-real-robot')


ARGUMENTS = [

    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),


    DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
                          [pkg_dis_tutorial3, 'maps', 'poligon.yaml']),
                          description='Full path to map yaml file to load'),

    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),

    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),

    
    # # A naj se to uporablja ali ne? Verjetno ne in ga damo na 'false', right?
    # Also. ni mi jasno kako je sim_turtlebot_nav.launch lahko bil kar brez tega.
    # Mogoče ker je naredil robot_spawn pred localization in temi stvarmi, torej je zagnal turtlebot4_spawn.launch,
    # ki pa definirat 'use_sim_time'
    DeclareLaunchArgument('use_sim_time', default_value='false',
                        choices=['true', 'false'],
                        description='use_sim_time'),
    
]


def generate_launch_description():

    # Create launch description
    ld = LaunchDescription(ARGUMENTS)


    # # Directories
    # package_dir = get_package_share_directory('RINS-task-real-robot')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    map_file = LaunchConfiguration('map')

    use_sim_time = LaunchConfiguration('use_sim_time')


    









    # Localization

    localization_launch = PathJoinSubstitution(
        [pkg_dis_tutorial3, 'launch', 'localization.launch.py'])
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
            ('map', map_file),
        ]
    )
    
    ld.add_action(localization)






    # Nav2

    nav2_launch = PathJoinSubstitution(
        [pkg_dis_tutorial3, 'launch', 'nav2.launch.py'])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('namespace', namespace)
        ]
    )
    ld.add_action(nav2)










    """Naslednji stvari sta iz turtlebot4_spawn.launch.py
    kjer se je zaganjal rviz, in laser filter
    in pa ignition simulation, kjer so se ustvarili razni nodei - to spustimo"""

    # rviz

    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')

    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('namespace', namespace)
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    ld.add_action(rviz)




    # laser filter

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
    ld.add_action(laser_filter)








    
    return ld
