#!/usr/bin/env python3
"""
Launch file for Gazebo Ignition simulation with the complete manipulation system
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSources
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package directories
    pkg_description = get_package_share_directory('manipulator_description')
    pkg_gazebo = get_package_share_directory('manipulator_gazebo')
    pkg_control = get_package_share_directory('manipulator_control')
    
    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'manipulator.urdf.xacro')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'manipulation.sdf')
    controllers_file = os.path.join(pkg_control, 'config', 'controllers.yaml')
    
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
            'use_sim_time': True
        }]
    )
    
    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen'
    )
    
    # Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'manipulation_system',
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for sensors
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    
    bridge_camera_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )
    
    bridge_depth_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )
    
    bridge_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )
    
    # Load joint_state_broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # Load arm_controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )
    
    # Load gripper_controller
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )
    
    # Event handlers to load controllers after robot is spawned
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[load_joint_state_broadcaster]
                )
            ]
        )
    )
    
    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[load_arm_controller]
                )
            ]
        )
    )
    
    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=4.0,
                    actions=[load_gripper_controller]
                )
            ]
        )
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        bridge_camera_info,
        bridge_camera_image,
        bridge_depth_image,
        bridge_points,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        load_gripper_controller_event
    ])
