#!/usr/bin/env python3
"""
sitl_circle_test.launch.py
啟動 3 架 real_drone_node (配合 PX4 SITL) + multi_drone_visualizer
用於測試畫圓任務。

使用方式：
  ros2 launch offboard_control_pkg sitl_circle_test.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drone_ids = [1, 2, 3]
    actions = []

    for did in drone_ids:
        actions.append(
            Node(
                package='offboard_control_pkg',
                executable='real_drone_node',
                name=f'real_drone_node_{did}',
                parameters=[{
                    'drone_id': did,
                    'require_gps': False,       # SITL 不需嚴格 GPS 檢查
                    'gps_eph_threshold': 10.0,   # 放寬 SITL 精度門檻
                }],
                output='screen'
            )
        )

    # Visualizer
    actions.append(
        Node(
            package='offboard_control_pkg',
            executable='multi_drone_visualizer',
            name='multi_drone_visualizer',
            parameters=[{'drone_ids': '1,2,3'}],
            output='screen'
        )
    )

    return LaunchDescription(actions)
