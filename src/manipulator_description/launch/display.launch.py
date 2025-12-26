#!/usr/bin/env python3
"""
Launch file for visualizing the manipulator in RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Get package directories
    pkg_description = get_package_share_directory('manipulator_description')
    
    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'manipulator.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'display.rviz')
    
    # Launch arguments
    use_gui = LaunchConfiguration('gui')
    use_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui'
    )
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher (without GUI)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=conditions.UnlessCondition(use_gui)
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=conditions.IfCondition(use_gui)
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
