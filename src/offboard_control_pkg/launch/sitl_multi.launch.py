#!/usr/bin/env python3
"""
sitl_multi.launch.py
- 啟動 signal_sim_node（環境模擬器）
- 啟動 swarm_controller_node（中央大腦）
- 為每台無人機啟動 drone_node（在各自命名空間下）
"""

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
import sys

def launch_setup(context, *args, **kwargs):
    """動態啟動各機的 drone_node"""
    
    drone_ids = [1, 2, 3]
    actions = []
    
    # 啟動環境模擬器
    actions.append(
        Node(
            package='offboard_control_pkg',
            executable='signal_sim_node',
            name='signal_sim_node',
            output='screen'
        )
    )
    
    # 啟動中央控制器
    actions.append(
        Node(
            package='offboard_control_pkg',
            executable='swarm_controller_node',
            name='swarm_controller_node',
            output='screen'
        )
    )
    
    # 為各無人機啟動 drone_node
    for did in drone_ids:
        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace(f'px4_{did}'),
                    Node(
                        package='offboard_control_pkg',
                        executable='drone_node',
                        name='drone_node',
                        parameters=[
                            {'drone_id': did}
                        ],
                        output='screen'
                    )
                ]
            )
        )
    
    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
