#!/usr/bin/env python3
"""
multi_drone_visualizer.py - 多機 RViz2 軌跡視覺化（GPS 共同參考框架）

訂閱每架無人機的 VehicleAttitude、VehicleLocalPosition、TrajectorySetpoint，
使用 GPS ref_lat/ref_lon/ref_alt 建立共同 ENU 參考框架，
在 RViz2 中以正確的相對位置顯示多機軌跡。
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, TrajectorySetpoint
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import json


def gps_offset_enu(ref_lat, ref_lon, ref_alt, lat0, lon0, alt0):
    """計算 (ref_lat, ref_lon, ref_alt) 相對於全局原點 (lat0, lon0, alt0) 的 ENU 偏移 (m)"""
    d_lat = math.radians(ref_lat - lat0)
    d_lon = math.radians(ref_lon - lon0)
    lat_avg = math.radians((ref_lat + lat0) / 2.0)
    R = 6371000.0
    east = d_lon * R * math.cos(lat_avg)
    north = d_lat * R
    up = ref_alt - alt0
    return (east, north, up)


class MultiDroneVisualizer(Node):
    def __init__(self):
        super().__init__('multi_drone_visualizer')

        self.declare_parameter('drone_ids', '1,2,3')
        ids_raw = self.get_parameter('drone_ids').get_parameter_value().string_value
        parsed_ids = []
        for token in ids_raw.split(','):
            token = token.strip()
            if token.isdigit():
                parsed_ids.append(int(token))
        self.ids = sorted(set(parsed_ids)) if parsed_ids else [1, 2, 3]
        self.get_logger().info(f"Visualizing drones: {self.ids}")

        # GPS 共同參考框架
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None
        self.gps_offset = {did: (0.0, 0.0, 0.0) for did in self.ids}

        # QoS 匹配 PX4 DDS bridge
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Circle reference data per drone
        self.circle_refs = {did: None for did in self.ids}

        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # 每架無人機獨立狀態與 publisher
        self.drones = {}
        self.pubs = {}

        for did in self.ids:
            ns = f'/px4_{did}'

            self.drones[did] = {
                'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
                'position_enu': np.array([0.0, 0.0, 0.0]),
                'velocity': np.array([0.0, 0.0, 0.0]),
                'setpoint_enu': np.array([0.0, 0.0, 0.0]),
                'vehicle_path': Path(),
                'setpoint_path': Path(),
            }
            self.drones[did]['vehicle_path'].header.frame_id = 'map'
            self.drones[did]['setpoint_path'].header.frame_id = 'map'

            viz_ns = f'/multi_drone_viz/drone_{did}'
            self.pubs[did] = {
                'vehicle_pose': self.create_publisher(PoseStamped, f'{viz_ns}/vehicle_pose', 10),
                'vehicle_path': self.create_publisher(Path, f'{viz_ns}/vehicle_path', 10),
                'setpoint_path': self.create_publisher(Path, f'{viz_ns}/setpoint_path', 10),
                'vehicle_velocity': self.create_publisher(Marker, f'{viz_ns}/vehicle_velocity', 10),
                'circle_reference': self.create_publisher(Marker, f'{viz_ns}/circle_reference', 10),
            }

            # 訂閱
            self.create_subscription(
                VehicleAttitude,
                f'{ns}/fmu/out/vehicle_attitude',
                lambda msg, d=did: self.attitude_cb(msg, d),
                px4_qos
            )
            self.create_subscription(
                VehicleLocalPosition,
                f'{ns}/fmu/out/vehicle_local_position',
                lambda msg, d=did: self.local_position_cb(msg, d),
                px4_qos
            )
            self.create_subscription(
                TrajectorySetpoint,
                f'{ns}/fmu/in/trajectory_setpoint',
                lambda msg, d=did: self.setpoint_cb(msg, d),
                px4_qos
            )
            self.create_subscription(
                String,
                f'{ns}/circle_ref',
                lambda msg, d=did: self.circle_ref_cb(msg, d),
                cmd_qos
            )

        # 20Hz publish timer
        self.create_timer(0.05, self.timer_callback)

    def attitude_cb(self, msg: VehicleAttitude, did: int):
        # PX4 NED quaternion [w, x, y, z] -> ENU: [w, x, -y, -z]
        q = msg.q
        self.drones[did]['attitude'] = np.array([q[0], q[1], -q[2], -q[3]])

    def local_position_cb(self, msg: VehicleLocalPosition, did: int):
        # GPS 共同參考框架設定
        if msg.xy_global and msg.ref_lat != 0.0:
            if self.origin_lat is None:
                self.origin_lat = msg.ref_lat
                self.origin_lon = msg.ref_lon
                self.origin_alt = msg.ref_alt
                self.get_logger().info(
                    f"Global origin set from drone {did}: "
                    f"lat={self.origin_lat:.7f}, lon={self.origin_lon:.7f}, alt={self.origin_alt:.2f}"
                )

            offset = gps_offset_enu(
                msg.ref_lat, msg.ref_lon, msg.ref_alt,
                self.origin_lat, self.origin_lon, self.origin_alt
            )
            self.gps_offset[did] = offset

        # NED -> ENU + GPS 偏移
        ox, oy, oz = self.gps_offset[did]
        enu_x = msg.y + ox    # NED.y -> ENU.x (East)
        enu_y = msg.x + oy    # NED.x -> ENU.y (North)
        enu_z = -msg.z + oz   # NED.z (down) -> ENU.z (up)
        self.drones[did]['position_enu'] = np.array([enu_x, enu_y, enu_z])

        # 速度 NED -> ENU
        self.drones[did]['velocity'] = np.array([msg.vy, msg.vx, -msg.vz])

    def setpoint_cb(self, msg: TrajectorySetpoint, did: int):
        pos = msg.position
        if len(pos) >= 3 and not (math.isnan(pos[0]) or math.isnan(pos[1]) or math.isnan(pos[2])):
            ox, oy, oz = self.gps_offset[did]
            enu_x = pos[1] + ox
            enu_y = pos[0] + oy
            enu_z = -pos[2] + oz
            self.drones[did]['setpoint_enu'] = np.array([enu_x, enu_y, enu_z])

            # 加入 setpoint path
            stamp = self.get_clock().now().to_msg()
            pt = PoseStamped()
            pt.header.stamp = stamp
            pt.header.frame_id = 'map'
            pt.pose.position.x = enu_x
            pt.pose.position.y = enu_y
            pt.pose.position.z = enu_z
            self.drones[did]['setpoint_path'].header.stamp = stamp
            self.drones[did]['setpoint_path'].poses.append(pt)

    def circle_ref_cb(self, msg: String, did: int):
        """Parse circle reference JSON and store for rendering."""
        try:
            data = json.loads(msg.data)
            if data.get("active", False):
                self.circle_refs[did] = data
            else:
                self.circle_refs[did] = None
        except Exception:
            pass

    def _publish_circle_marker(self, did, stamp):
        """Publish a cyan LINE_STRIP circle marker for the given drone's circle_ref."""
        ref = self.circle_refs.get(did)
        if ref is None:
            # Delete marker
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'map'
            marker.ns = f'drone_{did}_circle_ref'
            marker.id = did
            marker.action = Marker.DELETE
            self.pubs[did]['circle_reference'].publish(marker)
            return

        center_ned = ref.get("center_ned", [0, 0, -5])
        radius = ref.get("radius", 1.5)

        # NED -> ENU + GPS offset
        ox, oy, oz = self.gps_offset.get(did, (0.0, 0.0, 0.0))
        cx_enu = center_ned[1] + ox   # NED.y -> ENU.x
        cy_enu = center_ned[0] + oy   # NED.x -> ENU.y
        cz_enu = -center_ned[2] + oz  # NED.z -> ENU.z

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'map'
        marker.ns = f'drone_{did}_circle_ref'
        marker.id = did
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08  # line width

        # Cyan color
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        from geometry_msgs.msg import Point as GPoint
        num_points = 64
        for i in range(num_points + 1):
            angle = 2.0 * math.pi * i / num_points
            pt = GPoint()
            pt.x = cx_enu + radius * math.cos(angle)
            pt.y = cy_enu + radius * math.sin(angle)
            pt.z = cz_enu
            marker.points.append(pt)

        self.pubs[did]['circle_reference'].publish(marker)

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        for did in self.ids:
            state = self.drones[did]
            pubs = self.pubs[did]

            pos = state['position_enu']
            att = state['attitude']

            # PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = float(pos[0])
            pose_msg.pose.position.y = float(pos[1])
            pose_msg.pose.position.z = float(pos[2])
            pose_msg.pose.orientation.w = float(att[0])
            pose_msg.pose.orientation.x = float(att[1])
            pose_msg.pose.orientation.y = float(att[2])
            pose_msg.pose.orientation.z = float(att[3])
            pubs['vehicle_pose'].publish(pose_msg)

            # Vehicle Path
            path_pt = PoseStamped()
            path_pt.header.stamp = stamp
            path_pt.header.frame_id = 'map'
            path_pt.pose.position.x = float(pos[0])
            path_pt.pose.position.y = float(pos[1])
            path_pt.pose.position.z = float(pos[2])
            state['vehicle_path'].header.stamp = stamp
            state['vehicle_path'].poses.append(path_pt)
            pubs['vehicle_path'].publish(state['vehicle_path'])

            # Setpoint Path
            state['setpoint_path'].header.stamp = stamp
            pubs['setpoint_path'].publish(state['setpoint_path'])

            # Velocity Arrow Marker
            vel = state['velocity']
            vel_mag = float(np.linalg.norm(vel))
            if vel_mag > 0.01:
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = 'map'
                marker.ns = f'drone_{did}_velocity'
                marker.id = did
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                from geometry_msgs.msg import Point as GPoint
                start = GPoint()
                start.x = float(pos[0])
                start.y = float(pos[1])
                start.z = float(pos[2])
                end = GPoint()
                end.x = float(pos[0] + vel[0])
                end.y = float(pos[1] + vel[1])
                end.z = float(pos[2] + vel[2])
                marker.points = [start, end]

                marker.scale.x = 0.1  # shaft diameter
                marker.scale.y = 0.2  # head diameter
                marker.scale.z = 0.1  # head length

                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0

                pubs['vehicle_velocity'].publish(marker)

            # Circle reference marker
            self._publish_circle_marker(did, stamp)


def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
