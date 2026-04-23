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

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, TrajectorySetpoint, VehicleGlobalPosition
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from mission_interfaces.msg import GlobalMission, RelayStatus
from tf2_ros import TransformBroadcaster
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

        # Latched QoS — for mission waypoints / progress / handoffs so late-joining
        # Foxglove sessions still see them
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._latched_qos = latched_qos

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
                'navsat': self.create_publisher(NavSatFix, f'{viz_ns}/navsat', 10),
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
            self.create_subscription(
                VehicleGlobalPosition,
                f'{ns}/fmu/out/vehicle_global_position',
                lambda msg, d=did: self.global_position_cb(msg, d),
                px4_qos
            )

        # Mission waypoints / progress / handoff state
        self.mission_waypoints = []            # [(lat, lon, alt), ...]
        self.mission_wpt_publishers = []       # per-waypoint NavSatFix publishers (latched)
        self.current_waypoint_index = -1
        self.active_drone_id = 0
        self.prev_phase_per_drone = {}         # did -> last phase string
        self.handoff_points = []               # [(lat, lon), ...]
        self.handoff_publishers = []           # per-handoff NavSatFix publishers (latched)

        self.mission_markers_pub = self.create_publisher(
            MarkerArray, '/multi_drone_viz/mission/waypoints_markers', latched_qos
        )
        self.progress_pub = self.create_publisher(
            String, '/multi_drone_viz/mission/progress', latched_qos
        )

        self.create_subscription(
            GlobalMission, '/swarm/global_mission', self.global_mission_cb, cmd_qos
        )
        self.create_subscription(
            RelayStatus, '/swarm/relay_status', self.relay_status_cb, cmd_qos
        )

        # TF broadcaster: map -> drone_{did}_base_link
        self.tf_broadcaster = TransformBroadcaster(self)

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

    def global_position_cb(self, msg: VehicleGlobalPosition, did: int):
        """Re-publish drone GPS as NavSatFix for Foxglove Map panel."""
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'map'
        fix.latitude = float(msg.lat)
        fix.longitude = float(msg.lon)
        fix.altitude = float(msg.alt)
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pubs[did]['navsat'].publish(fix)

    def global_mission_cb(self, msg: GlobalMission):
        """Receive planned mission, publish each waypoint as latched NavSatFix and 3D markers."""
        n = min(len(msg.waypoint_lats), len(msg.waypoint_lons), len(msg.waypoint_alts))
        if n == 0:
            return
        self.mission_waypoints = [
            (float(msg.waypoint_lats[i]), float(msg.waypoint_lons[i]), float(msg.waypoint_alts[i]))
            for i in range(n)
        ]

        # Create/extend per-waypoint NavSatFix publishers for Foxglove Map panel
        while len(self.mission_wpt_publishers) < n:
            k = len(self.mission_wpt_publishers)
            self.mission_wpt_publishers.append(
                self.create_publisher(
                    NavSatFix, f'/multi_drone_viz/mission/waypoint_{k}', self._latched_qos
                )
            )

        stamp = self.get_clock().now().to_msg()
        for k, (lat, lon, alt) in enumerate(self.mission_waypoints):
            fix = NavSatFix()
            fix.header.stamp = stamp
            fix.header.frame_id = 'map'
            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = alt
            self.mission_wpt_publishers[k].publish(fix)

        self._publish_mission_markers()
        self.get_logger().info(
            f"[MISSION] id={msg.mission_id} waypoints={n} altitude={msg.mission_altitude:.1f}m"
        )

    def relay_status_cb(self, msg: RelayStatus):
        """Track progress, publish progress string, detect handoff events."""
        phase = msg.phase
        drone = int(msg.active_drone_id)
        idx = int(msg.current_index)
        total = int(msg.total_waypoints)

        self.active_drone_id = drone
        self.current_waypoint_index = idx

        progress = String()
        progress.data = f"Drone {drone} | WP {idx}/{total} | {phase}"
        self.progress_pub.publish(progress)

        # Re-publish mission markers with updated progress coloring
        self._publish_mission_markers()

        # Handoff detection: edge-triggered on entering RELAY_INTERRUPTED
        prev = self.prev_phase_per_drone.get(drone)
        if phase == 'RELAY_INTERRUPTED' and prev != 'RELAY_INTERRUPTED':
            self._record_handoff(msg.current_lat, msg.current_lon)
        self.prev_phase_per_drone[drone] = phase

    def _record_handoff(self, lat: float, lon: float):
        """Store a handoff point and publish it as a latched NavSatFix."""
        k = len(self.handoff_points)
        self.handoff_points.append((float(lat), float(lon)))
        pub = self.create_publisher(
            NavSatFix, f'/multi_drone_viz/handoffs/point_{k}', self._latched_qos
        )
        self.handoff_publishers.append(pub)
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'map'
        fix.latitude = float(lat)
        fix.longitude = float(lon)
        fix.altitude = 0.0
        pub.publish(fix)
        self.get_logger().info(f"[HANDOFF] #{k} at ({lat:.7f}, {lon:.7f})")

    def _publish_mission_markers(self):
        """MarkerArray for 3D panel: waypoints spheres + route line + handoff markers.
        Requires self.origin_lat to be set (from any drone's local_position_cb)."""
        if self.origin_lat is None or not self.mission_waypoints:
            return

        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mission_z = 2.0  # display height in 3D panel (visual only)

        for k, (lat, lon, alt) in enumerate(self.mission_waypoints):
            east, north, _up = gps_offset_enu(
                lat, lon, alt, self.origin_lat, self.origin_lon, self.origin_alt
            )
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'map'
            m.ns = 'mission_waypoints'
            m.id = k
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(east)
            m.pose.position.y = float(north)
            m.pose.position.z = mission_z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.2
            if k < self.current_waypoint_index:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.8  # done
            elif k == self.current_waypoint_index:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0  # current
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 0.6  # pending
            arr.markers.append(m)

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = 'map'
        line.ns = 'mission_route'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.2
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 0.5, 1.0, 0.7
        for (lat, lon, alt) in self.mission_waypoints:
            east, north, _up = gps_offset_enu(
                lat, lon, alt, self.origin_lat, self.origin_lon, self.origin_alt
            )
            pt = Point()
            pt.x = float(east)
            pt.y = float(north)
            pt.z = mission_z
            line.points.append(pt)
        arr.markers.append(line)

        for k, (lat, lon) in enumerate(self.handoff_points):
            east, north, _up = gps_offset_enu(
                lat, lon, self.origin_alt, self.origin_lat, self.origin_lon, self.origin_alt
            )
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = 'map'
            m.ns = 'mission_handoffs'
            m.id = k
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(east)
            m.pose.position.y = float(north)
            m.pose.position.z = mission_z / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.8
            m.scale.z = mission_z
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.5, 0.9
            arr.markers.append(m)

        self.mission_markers_pub.publish(arr)

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

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = f'drone_{did}_base_link'
            tf_msg.transform.translation.x = float(pos[0])
            tf_msg.transform.translation.y = float(pos[1])
            tf_msg.transform.translation.z = float(pos[2])
            tf_msg.transform.rotation.w = float(att[0])
            tf_msg.transform.rotation.x = float(att[1])
            tf_msg.transform.rotation.y = float(att[2])
            tf_msg.transform.rotation.z = float(att[3])
            self.tf_broadcaster.sendTransform(tf_msg)

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
