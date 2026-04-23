#!/usr/bin/env python3
"""
sitl_multi.launch.py — SITL 用，為三台無人機啟動 real_drone_node (sitl_mode=true)。
（signal_sim_node / swarm_controller 已移除，按需手動起）
"""
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    actions = []
    for did in [1, 2, 3]:
        actions.append(
            GroupAction(actions=[
                PushRosNamespace(f'px4_{did}'),
                Node(
                    package='offboard_control_pkg',
                    executable='real_drone_node',
                    name='drone_node',
                    parameters=[{'drone_id': did, 'sitl_mode': True}],
                    output='screen',
                ),
            ])
        )
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
