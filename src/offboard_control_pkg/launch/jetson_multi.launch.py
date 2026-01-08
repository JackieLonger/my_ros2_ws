#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='1', description='PX4 drone ID (integer)'
    )
    start_fast_scan_arg = DeclareLaunchArgument(
        'start_fast_scan', default_value='true', description='Start fast_scan_node'
    )
    tracker_a_arg = DeclareLaunchArgument(
        'tracker_a_id', default_value='!e2e5b7c4', description='Meshtastic Tracker A ID'
    )
    tracker_b_arg = DeclareLaunchArgument(
        'tracker_b_id', default_value='!e2e5b8f8', description='Meshtastic Tracker B ID'
    )
    meshtastic_port_arg = DeclareLaunchArgument(
        'meshtastic_port', default_value='/dev/ttyACM0', description='Meshtastic serial port'
    )

    drone_id = LaunchConfiguration('drone_id')
    start_fast_scan = LaunchConfiguration('start_fast_scan')
    tracker_a_id = LaunchConfiguration('tracker_a_id')
    tracker_b_id = LaunchConfiguration('tracker_b_id')
    meshtastic_port = LaunchConfiguration('meshtastic_port')

    # Namespace based on drone id
    ns = [TextSubstitution(text='px4_'), drone_id]

    # Group actions under namespace /px4_N
    group = GroupAction([
        PushRosNamespace(ns),
        
        # Drone control node
        Node(
            package='offboard_control_pkg',
            executable='drone_node',
            name='drone_control_node',
            parameters=[{'drone_id': drone_id}],
            output='screen'
        ),
        
        # Fast scan node (optional)
        Node(
            package='offboard_control_pkg',
            executable='fast_scan_node',
            name='fast_scan_node',
            parameters=[
                {'tracker_a_id': ParameterValue(tracker_a_id, value_type=str),
                 'tracker_b_id': ParameterValue(tracker_b_id, value_type=str),
                 'meshtastic_port': ParameterValue(meshtastic_port, value_type=str)}
            ],
            condition=IfCondition(start_fast_scan),
            output='screen'
        )
    ])

    return LaunchDescription([
        drone_id_arg,
        start_fast_scan_arg,
        tracker_a_arg,
        tracker_b_arg,
        meshtastic_port_arg,
        group
    ])
