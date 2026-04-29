#!/usr/bin/env python3
"""
real_drone_node.py - 統一 SITL/實機 PX4 Offboard 控制 + 多機接力任務

功能：
1. PX4 Offboard Position Control (20Hz TrajectorySetpoint)
2. CircleMission Action Server（協作畫圓）
3. 接力任務：GPS 航點執行、RelayHandoff Service（drone-to-drone 直接交接）
4. sitl_mode 參數統一 SITL/實機（QoS、GPS 檢查等）
5. 高度分層碰撞防護（任務層 vs 返航層）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Point
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                           VehicleStatus, VehicleGlobalPosition, VehicleLocalPosition)
from mission_interfaces.action import CircleMission
from mission_interfaces.srv import RelayHandoff
from mission_interfaces.msg import GlobalMission, RelayStatus
import math
import json
import time
import threading


# ─── GPS / Coordinate Helpers ───────────────────────────────────────────────

def gps_offset_enu(ref_lat, ref_lon, ref_alt, lat0, lon0, alt0):
    """Compute ENU offset (east, north, up) in metres from (lat0,lon0,alt0) to (ref_lat,ref_lon,ref_alt)."""
    d_lat = math.radians(ref_lat - lat0)
    d_lon = math.radians(ref_lon - lon0)
    lat_avg = math.radians((ref_lat + lat0) / 2.0)
    R = 6371000.0
    east = d_lon * R * math.cos(lat_avg)
    north = d_lat * R
    up = ref_alt - alt0
    return (east, north, up)


def gps_to_local_ned(target_lat, target_lon, target_alt, ref_lat, ref_lon, ref_alt):
    """Convert GPS (AMSL) to local NED relative to the drone's EKF origin."""
    east, north, up = gps_offset_enu(target_lat, target_lon, target_alt,
                                      ref_lat, ref_lon, ref_alt)
    return (north, east, -up)  # ENU → NED


# ─── Main Node ──────────────────────────────────────────────────────────────

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('sitl_mode', False)
        self.declare_parameter('require_gps', True)
        self.declare_parameter('gps_eph_threshold', 2.5)
        self.declare_parameter('next_drone_id', 0)        # 0 = last drone, no handoff
        self.declare_parameter('mission_altitude', 10.0)   # metres (relative)
        self.declare_parameter('return_altitude', 15.0)    # metres (relative)
        self.declare_parameter('cruise_speed', 1.0)        # m/s max horizontal speed

        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self.sitl_mode = self.get_parameter('sitl_mode').get_parameter_value().bool_value
        self.next_drone_id = self.get_parameter('next_drone_id').get_parameter_value().integer_value
        self.mission_altitude = self.get_parameter('mission_altitude').get_parameter_value().double_value
        self.return_altitude = self.get_parameter('return_altitude').get_parameter_value().double_value
        self.cruise_speed = self.get_parameter('cruise_speed').get_parameter_value().double_value
        self.target_ns = f'px4_{self.drone_id}'

        # sitl_mode overrides
        if self.sitl_mode:
            self.require_gps = False
            self.gps_eph_threshold = 10.0
            self.get_logger().info(f"[{self.target_ns}] === SITL 模式 ===")
        else:
            self.require_gps = self.get_parameter('require_gps').get_parameter_value().bool_value
            self.gps_eph_threshold = self.get_parameter('gps_eph_threshold').get_parameter_value().double_value
            self.get_logger().info(f"[{self.target_ns}] === 實機 Offboard 控制 ===")

        self.get_logger().info(
            f"[{self.target_ns}] Drone ID: {self.drone_id}, "
            f"Next: {self.next_drone_id}, "
            f"Mission Alt: {self.mission_altitude}m, Return Alt: {self.return_altitude}m")

        # ── QoS ─────────────────────────────────────────────────────────
        if self.sitl_mode:
            self.px4_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST, depth=1)
        else:
            self.px4_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=1)

        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── PX4 Publishers ──────────────────────────────────────────────
        self.offboard_ctrl_pub = self.create_publisher(
            OffboardControlMode, f'/{self.target_ns}/fmu/in/offboard_control_mode', self.px4_qos)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, f'/{self.target_ns}/fmu/in/trajectory_setpoint', self.px4_qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f'/{self.target_ns}/fmu/in/vehicle_command', self.px4_qos)

        # ── PX4 Subscribers ─────────────────────────────────────────────
        self.create_subscription(VehicleStatus,
            f'/{self.target_ns}/fmu/out/vehicle_status', self.status_cb, self.px4_qos)
        self.create_subscription(VehicleGlobalPosition,
            f'/{self.target_ns}/fmu/out/vehicle_global_position', self.global_pos_cb, self.px4_qos)
        self.create_subscription(VehicleLocalPosition,
            f'/{self.target_ns}/fmu/out/vehicle_local_position', self.local_pos_cb, self.px4_qos)

        # ── Laptop Commands ─────────────────────────────────────────────
        self.create_subscription(String, f'/{self.target_ns}/laptop/action', self.action_cb, cmd_qos)
        self.create_subscription(Point, f'/{self.target_ns}/laptop/setpoint', self.setpoint_cb, cmd_qos)
        self.create_subscription(String, f'/{self.target_ns}/laptop/scan_action', self.scan_action_cb, cmd_qos)
        self.create_subscription(Float64, f'/{self.target_ns}/laptop/yaw_setpoint', self.yaw_setpoint_cb, cmd_qos)

        # ── External control gate ───────────────────────────────────────
        self.external_control_active = False
        self.create_subscription(Bool, f'/{self.target_ns}/external_control', self.external_control_cb, cmd_qos)

        # ── Scan Components ─────────────────────────────────────────────
        self.scan_control_pub = self.create_publisher(Bool, f'/{self.target_ns}/scan_control', cmd_qos)
        self.scan_ready_pub = self.create_publisher(String, f'/{self.target_ns}/scan_ready', cmd_qos)
        self.create_subscription(String, f'/{self.target_ns}/link_quality', self.link_quality_cb, cmd_qos)

        # ── Relay Mission ───────────────────────────────────────────────
        self.create_subscription(Bool, f'/{self.target_ns}/relay_interrupt', self.relay_interrupt_cb, cmd_qos)
        self.create_subscription(GlobalMission, '/swarm/global_mission', self.global_mission_cb, cmd_qos)
        self.relay_status_pub = self.create_publisher(RelayStatus, '/swarm/relay_status', cmd_qos)
        self.map_received_pub = self.create_publisher(String, '/swarm/map_received', cmd_qos)

        # RelayHandoff Service server (other drones call this to hand off)
        self.relay_srv = self.create_service(
            RelayHandoff, f'/{self.target_ns}/relay_handoff', self.relay_handoff_cb)

        # ── State Variables ─────────────────────────────────────────────
        self.target_pos_ned = [0.0, 0.0, 0.0]
        self.target_yaw_ned = float('nan')
        self.origin_ref_ned = [0.0, 0.0, 0.0]
        self.current_local_ned = [0.0, 0.0, 0.0]

        self.is_connected = False
        self.pre_flight_checks_pass = False
        self.arming_state = 0
        self.px4_system_id = 1
        self.got_global_pos = False
        self.current_eph = 999.9
        self.current_gps_lat = 0.0
        self.current_gps_lon = 0.0
        self.current_gps_alt = 0.0

        # EKF reference (local frame origin in GPS)
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.ref_alt = 0.0
        self.ref_ready = False

        self.scan_results = {}
        self.expected_trackers = ["!e2e5b7c4", "!e2e5b8f8"]
        self.scan_completed = False
        self.status_check_count = 0

        # ── Relay Mission State ─────────────────────────────────────────
        self.relay_state = 'IDLE'   # IDLE / RELAY_IDLE / RELAY_TAKEOFF / RELAY_GOTO_HANDOFF / RELAY_EXECUTING / RELAY_INTERRUPTED / RELAY_RETURNING
        self.paused = False  # FORCE_HOVER 暫停 → state machine 不推進，按 RESUME_RELAY 解除
        self.mission_waypoints_gps = []     # list of (lat, lon, alt_amsl)
        self.mission_waypoints_ned = []     # list of (x, y, z) in local NED
        self.mission_converted = False
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.handoff_target_ned = None      # NED position of handoff point
        self.relay_home_ned = None          # NED position to return to
        self.waypoint_reach_threshold = 1.5  # metres

        # ── Circle Mission Action Server ────────────────────────────────
        self.action_cb_group = ReentrantCallbackGroup()
        self.circle_action_server = ActionServer(
            self, CircleMission, f'/{self.target_ns}/circle_mission/execute',
            execute_callback=self.circle_execute_cb,
            goal_callback=self.circle_goal_cb,
            cancel_callback=self.circle_cancel_cb,
            callback_group=self.action_cb_group)

        self.circle_state = 'IDLE'
        self.circle_cancel_requested = False
        self.circle_target_id = None
        self.circle_center_ned = None
        self.circle_radius = 1.5
        self.circle_omega = 0.5
        self.circle_start_time = None
        self.circle_phase_start_time = None
        self.circle_timer = None
        self.circle_target_local_pos = None

        self.circle_target_pos_sub = None
        self.circle_target_offboard_pub = None
        self.circle_target_setpoint_pub = None
        self.circle_target_ext_ctrl_pub = None
        self.circle_ref_pub = self.create_publisher(String, f'/{self.target_ns}/circle_ref', cmd_qos)

        # ── 20Hz Main Timer ─────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"[{self.target_ns}] 等待 PX4 連線...")

    # ═══════════════════════════════════════════════════════════════════════
    #  PX4 Callbacks
    # ═══════════════════════════════════════════════════════════════════════

    def status_cb(self, msg):
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.is_connected = True
        try:
            self.arming_state = int(msg.arming_state)
            if hasattr(msg, 'system_id') and msg.system_id > 0:
                self.px4_system_id = int(msg.system_id)
        except Exception:
            pass
        self.status_check_count += 1
        if self.status_check_count % 40 == 0:
            armed_str = "ARMED" if self.arming_state == 2 else "DISARMED"
            checks_str = "PASS" if self.pre_flight_checks_pass else "FAIL"
            self.get_logger().info(f"[{self.target_ns}] 狀態: {armed_str} | Preflight: {checks_str}")

    def global_pos_cb(self, msg):
        self.got_global_pos = True
        self.current_eph = msg.eph
        self.current_gps_lat = msg.lat
        self.current_gps_lon = msg.lon
        self.current_gps_alt = msg.alt

    def local_pos_cb(self, msg: VehicleLocalPosition):
        try:
            self.current_local_ned = [float(msg.x), float(msg.y), float(msg.z)]
        except Exception:
            pass
        # Capture EKF reference origin for GPS→local conversion
        if not self.ref_ready and msg.xy_global and msg.ref_lat != 0.0:
            self.ref_lat = msg.ref_lat
            self.ref_lon = msg.ref_lon
            self.ref_alt = msg.ref_alt
            self.ref_ready = True
            self.get_logger().info(
                f"[{self.target_ns}] EKF ref: lat={self.ref_lat:.7f}, "
                f"lon={self.ref_lon:.7f}, alt={self.ref_alt:.2f}")
            # If we have pending GPS waypoints, convert them now
            if self.mission_waypoints_gps and not self.mission_converted:
                self._convert_mission_waypoints()

    # ═══════════════════════════════════════════════════════════════════════
    #  Laptop Command Callbacks (existing functionality)
    # ═══════════════════════════════════════════════════════════════════════

    def yaw_setpoint_cb(self, msg):
        self.target_yaw_ned = float(msg.data)

    def setpoint_cb(self, msg):
        # ENU → NED
        self.target_pos_ned = [msg.y, msg.x, -msg.z]

    def action_cb(self, msg):
        cmd = msg.data.upper()
        if cmd == "TAKEOFF_CHECK":
            self.paused = False
            self.perform_safety_takeoff()
        elif cmd == "LAND":
            self.get_logger().info(f"[{self.target_ns}] 執行降落...")
            # 凍結 state machine（target_pos_ned 不再被推進）但保留 relay_state
            # → 使用者後續按 X 仍能觸發中斷+交接給下一台
            self.paused = True
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif cmd == "RETURN_HOME":
            self.get_logger().info(f"[{self.target_ns}] *** 返航 ***")
            self.paused = False
            if self.relay_home_ned is None:
                self.relay_home_ned = [0.0, 0.0, 0.0]  # fallback: 用 origin 作為 home
            self.relay_state = 'RELAY_RETURNING'
        elif cmd == "DISARM":
            self.get_logger().info(f"[{self.target_ns}] 強制上鎖...")
            self.paused = False
            self.relay_state = 'IDLE'
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        elif cmd == "RESET_ORIGIN":
            self.origin_ref_ned = list(self.current_local_ned)
            self.target_yaw_ned = float('nan')
            self.paused = False
            self.relay_state = 'IDLE'
            self.get_logger().info(f"[{self.target_ns}] 重置相對原點，relay_state=IDLE")
        elif cmd == "FORCE_HOVER":
            self.get_logger().info(f"[{self.target_ns}] *** 緊急懸停（暫停接力進度） ***")
            self.target_pos_ned = list(self.current_local_ned)
            self.target_yaw_ned = float('nan')
            self.paused = True  # 阻止 relay state machine 推進，但保留 relay_state
            self.external_control_active = False
            if self.circle_state != 'IDLE':
                self.circle_cancel_requested = True
        elif cmd == "RESUME_RELAY":
            if self.paused:
                self.get_logger().info(f"[{self.target_ns}] *** 恢復暫停接力任務 ***")
            self.paused = False
        elif cmd == "START_RELAY":
            self.paused = False
            self._start_relay_mission()

    def scan_action_cb(self, msg):
        cmd = msg.data.upper()
        if cmd == "START_SCAN":
            self.scan_results = {}
            self.scan_completed = False
            out = Bool(); out.data = True
            self.scan_control_pub.publish(out)
        elif cmd == "STOP_SCAN":
            out = Bool(); out.data = False
            self.scan_control_pub.publish(out)

    def link_quality_cb(self, msg):
        try:
            data = json.loads(msg.data)
            target_id = data.get('target_id')
            status = data.get('status')
            self.scan_results[target_id] = status
            if all(t in self.scan_results for t in self.expected_trackers):
                self.scan_completed = True
                out = String()
                out.data = json.dumps({"status": "completed", "results": self.scan_results})
                self.scan_ready_pub.publish(out)
        except Exception:
            pass

    def external_control_cb(self, msg):
        prev = self.external_control_active
        self.external_control_active = msg.data
        if msg.data and not prev:
            self.get_logger().info(f"[{self.target_ns}] External control ACTIVE")
        elif not msg.data and prev:
            self.get_logger().info(f"[{self.target_ns}] External control RELEASED")

    # ═══════════════════════════════════════════════════════════════════════
    #  Relay Mission: GlobalMission, Interrupt, Handoff
    # ═══════════════════════════════════════════════════════════════════════

    def global_mission_cb(self, msg: GlobalMission):
        """Receive global mission waypoints from ground station."""
        n = len(msg.waypoint_lats)
        if n == 0:
            self.get_logger().warn(f"[{self.target_ns}] 收到空的 GlobalMission")
            return

        self.mission_waypoints_gps = []
        for i in range(n):
            self.mission_waypoints_gps.append((
                msg.waypoint_lats[i], msg.waypoint_lons[i], msg.waypoint_alts[i]))
        self.total_waypoints = n
        self.mission_altitude = msg.mission_altitude if msg.mission_altitude > 0 else self.mission_altitude
        self.mission_converted = False

        self.get_logger().info(
            f"[{self.target_ns}] 收到 GlobalMission: {n} 航點, "
            f"mission_alt={self.mission_altitude}m, id={msg.mission_id}")

        # Convert if EKF ref is already ready
        if self.ref_ready:
            self._convert_mission_waypoints()

        # Report map received
        ack = String()
        ack.data = json.dumps({"drone_id": self.drone_id, "waypoints": n})
        self.map_received_pub.publish(ack)

    def _convert_mission_waypoints(self):
        """Batch-convert GPS waypoints to local NED using current EKF reference.
        Only convert horizontal (lat/lon → x/y). Altitude uses mission_altitude parameter."""
        self.mission_waypoints_ned = []
        for lat, lon, _alt in self.mission_waypoints_gps:
            ned = gps_to_local_ned(lat, lon, self.ref_alt, self.ref_lat, self.ref_lon, self.ref_alt)
            # Override Z with mission altitude (NED: negative = up)
            self.mission_waypoints_ned.append((ned[0], ned[1], -self.mission_altitude))
        self.mission_converted = True
        self.relay_state = 'RELAY_IDLE'
        self.get_logger().info(
            f"[{self.target_ns}] 航點已轉換為 local NED ({len(self.mission_waypoints_ned)} 點)")

    def relay_interrupt_cb(self, msg: Bool):
        """Ground station triggers interruption."""
        interruptible = ('RELAY_TAKEOFF', 'RELAY_GOTO_HANDOFF', 'RELAY_EXECUTING')
        if msg.data and self.relay_state in interruptible:
            self.get_logger().info(f"[{self.target_ns}] *** 收到中斷指令 (state={self.relay_state}) ***")
            self.paused = False  # 解除暫停讓 state machine 進入 INTERRUPTED 流程
            self.relay_state = 'RELAY_INTERRUPTED'

    def relay_handoff_cb(self, request, response):
        """Service callback: another drone asks us to take over the mission."""
        self.get_logger().info(
            f"[{self.target_ns}] 收到交接請求: from drone {request.from_drone_id}, "
            f"GPS=({request.handoff_lat:.7f}, {request.handoff_lon:.7f}), "
            f"start_index={request.start_index}")

        if not self.mission_converted:
            response.accepted = False
            response.reason = "Mission waypoints not ready"
            self.get_logger().error(f"[{self.target_ns}] 拒絕交接: 航點未就緒")
            return response

        if request.start_index >= self.total_waypoints:
            response.accepted = False
            response.reason = "start_index out of range"
            return response

        # Convert handoff GPS to our local NED
        if self.ref_ready:
            self.handoff_target_ned = gps_to_local_ned(
                request.handoff_lat, request.handoff_lon, request.handoff_alt,
                self.ref_lat, self.ref_lon, self.ref_alt)
            # Override Z with mission altitude (NED: negative = up)
            self.handoff_target_ned = (
                self.handoff_target_ned[0],
                self.handoff_target_ned[1],
                -self.mission_altitude)
        else:
            response.accepted = False
            response.reason = "EKF reference not ready"
            return response

        self.current_waypoint_index = request.start_index
        response.accepted = True
        response.reason = "Accepted"

        self.get_logger().info(
            f"[{self.target_ns}] 交接已接受: 飛往 NED "
            f"({self.handoff_target_ned[0]:.1f}, {self.handoff_target_ned[1]:.1f}, "
            f"{self.handoff_target_ned[2]:.1f}), 從 index {self.current_waypoint_index} 繼續")

        # Start relay mission in background (service must return quickly)
        threading.Thread(target=self._start_relay_as_receiver, daemon=True).start()
        return response

    def _start_relay_mission(self):
        """First drone starts the relay mission from index 0."""
        if not self.mission_converted:
            self.get_logger().error(f"[{self.target_ns}] 無法啟動接力: 航點未就緒")
            return
        if self.relay_state not in ('IDLE', 'RELAY_IDLE'):
            self.get_logger().warn(f"[{self.target_ns}] 接力任務已在執行中 (state={self.relay_state})，忽略")
            return
        self.current_waypoint_index = 0
        self.handoff_target_ned = None  # No handoff point for first drone
        self.relay_home_ned = list(self.current_local_ned)  # Remember takeoff position
        self.relay_state = 'RELAY_TAKEOFF'
        # Set target to mission altitude directly above current position
        self.target_pos_ned = [
            self.current_local_ned[0],
            self.current_local_ned[1],
            -self.mission_altitude]
        self.get_logger().info(f"[{self.target_ns}] 接力任務啟動: 起飛到 {self.mission_altitude}m")
        # Arm and switch to offboard
        self.perform_safety_takeoff()

    def _start_relay_as_receiver(self):
        """Called by relay_handoff_cb in a thread. Performs takeoff and starts mission."""
        self.relay_home_ned = list(self.current_local_ned)  # Remember takeoff position
        self.relay_state = 'RELAY_TAKEOFF'
        self.target_pos_ned = [
            self.current_local_ned[0],
            self.current_local_ned[1],
            -self.mission_altitude]
        self.get_logger().info(f"[{self.target_ns}] 接力接收: 起飛到 {self.mission_altitude}m")
        self.perform_safety_takeoff()

    def _do_relay_handoff(self):
        """Interrupted drone hands off to next drone via Service call."""
        if self.next_drone_id == 0:
            self.get_logger().info(f"[{self.target_ns}] 最後一架，無需交接，直接返航")
            self._start_return()
            return

        next_ns = f'px4_{self.next_drone_id}'
        srv_name = f'/{next_ns}/relay_handoff'
        self.get_logger().info(
            f"[{self.target_ns}] 呼叫 {srv_name}: GPS=({self.current_gps_lat:.7f}, "
            f"{self.current_gps_lon:.7f}), index={self.current_waypoint_index}")

        client = self.create_client(RelayHandoff, srv_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"[{self.target_ns}] Service {srv_name} 不可用，直接返航")
            self._start_return()
            return

        req = RelayHandoff.Request()
        req.from_drone_id = self.drone_id
        req.handoff_lat = self.current_gps_lat
        req.handoff_lon = self.current_gps_lon
        req.handoff_alt = self.current_gps_alt
        req.start_index = self.current_waypoint_index

        future = client.call_async(req)
        # Wait for response (with timeout)
        start = time.time()
        while not future.done() and (time.time() - start) < 10.0:
            time.sleep(0.1)

        if future.done():
            result = future.result()
            if result and result.accepted:
                self.get_logger().info(f"[{self.target_ns}] 交接成功: {result.reason}")
            else:
                reason = result.reason if result else "No response"
                self.get_logger().warn(f"[{self.target_ns}] 交接被拒絕: {reason}")
        else:
            self.get_logger().error(f"[{self.target_ns}] 交接超時")

        self.destroy_client(client)
        self._start_return()

    def _start_return(self):
        """Begin return-to-home: climb to return_altitude then fly back."""
        self.relay_state = 'RELAY_RETURNING'
        if self.relay_home_ned:
            # First climb to return altitude, then fly home
            self.target_pos_ned = [
                self.current_local_ned[0],
                self.current_local_ned[1],
                -self.return_altitude]
        else:
            self.target_pos_ned = [0.0, 0.0, -self.return_altitude]
        self.get_logger().info(f"[{self.target_ns}] 返航: 爬升到 {self.return_altitude}m")

    def _publish_relay_status(self):
        """Publish relay status for Foxglove monitoring."""
        msg = RelayStatus()
        msg.active_drone_id = self.drone_id
        msg.phase = self.relay_state
        msg.current_index = self.current_waypoint_index
        msg.total_waypoints = self.total_waypoints
        msg.current_lat = self.current_gps_lat
        msg.current_lon = self.current_gps_lon
        self.relay_status_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    #  20Hz Timer — Main Control Loop
    # ═══════════════════════════════════════════════════════════════════════

    def timer_callback(self):
        # Publish relay status only when actively doing something (not IDLE/RELAY_IDLE)
        if self.relay_state not in ('IDLE', 'RELAY_IDLE'):
            self._publish_relay_status()

        # Relay state machine（paused 時不推進航點，target_pos_ned 維持當前位置）
        if not self.paused:
            if self.relay_state == 'RELAY_TAKEOFF':
                self._handle_relay_takeoff()
            elif self.relay_state == 'RELAY_GOTO_HANDOFF':
                self._handle_relay_goto_handoff()
            elif self.relay_state == 'RELAY_EXECUTING':
                self._handle_relay_executing()
            elif self.relay_state == 'RELAY_INTERRUPTED':
                self._handle_relay_interrupted()
            elif self.relay_state == 'RELAY_RETURNING':
                self._handle_relay_returning()

        # Publish offboard heartbeat + setpoint (unless externally controlled)
        if not self.external_control_active:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(
                self.target_pos_ned[0], self.target_pos_ned[1], self.target_pos_ned[2])

    def _handle_relay_takeoff(self):
        """Wait until drone reaches mission altitude."""
        current_alt = -self.current_local_ned[2]  # NED z negative = up
        if current_alt >= self.mission_altitude - 1.0:
            if self.handoff_target_ned:
                # Receiver drone: fly to handoff point
                self.relay_state = 'RELAY_GOTO_HANDOFF'
                self.target_pos_ned = list(self.handoff_target_ned)
                self.get_logger().info(f"[{self.target_ns}] 到達任務高度，飛往交接點")
            else:
                # First drone: start executing from index 0
                self.relay_state = 'RELAY_EXECUTING'
                if self.mission_waypoints_ned:
                    wp = self.mission_waypoints_ned[0]
                    self.target_pos_ned = [wp[0], wp[1], -self.mission_altitude]
                self.get_logger().info(f"[{self.target_ns}] 到達任務高度，開始執行航點")

    def _handle_relay_goto_handoff(self):
        """Fly to handoff GPS point, then start executing waypoints."""
        dist = self._distance_to_target_2d()
        if dist < self.waypoint_reach_threshold:
            self.relay_state = 'RELAY_EXECUTING'
            if self.current_waypoint_index < len(self.mission_waypoints_ned):
                wp = self.mission_waypoints_ned[self.current_waypoint_index]
                self.target_pos_ned = [wp[0], wp[1], -self.mission_altitude]
            self.get_logger().info(
                f"[{self.target_ns}] 到達交接點，從 index {self.current_waypoint_index} 繼續執行")

    def _handle_relay_executing(self):
        """Follow waypoints sequentially."""
        if self.current_waypoint_index >= len(self.mission_waypoints_ned):
            self.get_logger().info(f"[{self.target_ns}] 所有航點完成！返航")
            self._start_return()
            return

        dist = self._distance_to_target_2d()

        # Debug log every 2 seconds (40 ticks at 20Hz)
        self.status_check_count += 1
        if self.status_check_count % 40 == 0:
            cx, cy, cz = self.current_local_ned
            tx, ty, tz = self.target_pos_ned
            self.get_logger().info(
                f"[{self.target_ns}] EXEC wp={self.current_waypoint_index} "
                f"pos=({cx:.1f},{cy:.1f},{cz:.1f}) "
                f"tgt=({tx:.1f},{ty:.1f},{tz:.1f}) dist={dist:.1f}m")

        if dist < self.waypoint_reach_threshold:
            self.get_logger().info(
                f"[{self.target_ns}] 到達航點 {self.current_waypoint_index}/{self.total_waypoints}")
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.mission_waypoints_ned):
                wp = self.mission_waypoints_ned[self.current_waypoint_index]
                self.target_pos_ned = [wp[0], wp[1], -self.mission_altitude]

    def _handle_relay_interrupted(self):
        """Brake, read GPS, call next drone, then return."""
        # Brake: hold current position
        self.target_pos_ned = list(self.current_local_ned)
        self.get_logger().info(
            f"[{self.target_ns}] 懸停中 GPS=({self.current_gps_lat:.7f}, "
            f"{self.current_gps_lon:.7f}), 準備交接...")
        # Do handoff in background thread so 20Hz loop continues
        self.relay_state = 'RELAY_HANDOFF_PENDING'  # Prevent re-entry
        threading.Thread(target=self._do_relay_handoff, daemon=True).start()

    def _handle_relay_returning(self):
        """Climb to return altitude, then fly home, then land."""
        current_alt = -self.current_local_ned[2]
        if current_alt >= self.return_altitude - 1.0:
            if self.relay_home_ned:
                home_dist = math.sqrt(
                    (self.current_local_ned[0] - self.relay_home_ned[0])**2 +
                    (self.current_local_ned[1] - self.relay_home_ned[1])**2)
                if home_dist > 1.0:
                    # Fly home at return altitude
                    self.target_pos_ned = [
                        self.relay_home_ned[0],
                        self.relay_home_ned[1],
                        -self.return_altitude]
                else:
                    # Arrived home, land
                    self.get_logger().info(f"[{self.target_ns}] 到達起飛點上方，執行降落")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.relay_state = 'IDLE'

    def _distance_to_target_2d(self):
        """2D horizontal distance from current position to target."""
        dx = self.current_local_ned[0] - self.target_pos_ned[0]
        dy = self.current_local_ned[1] - self.target_pos_ned[1]
        return math.sqrt(dx*dx + dy*dy)

    # ═══════════════════════════════════════════════════════════════════════
    #  Takeoff
    # ═══════════════════════════════════════════════════════════════════════

    def perform_safety_takeoff(self):
        self.get_logger().info(f"[{self.target_ns}] ===== 起飛檢查 =====")

        if not self.is_connected:
            self.get_logger().error("未連接 PX4")
            return
        if not self.pre_flight_checks_pass:
            self.get_logger().error("Preflight Checks 失敗")
            return

        if self.require_gps:
            if not self.got_global_pos or self.current_eph > self.gps_eph_threshold:
                self.get_logger().error(f"GPS 精度不足: {self.current_eph:.2f}m")
                return

        is_armed = (self.arming_state == 2)
        sleep_time = 0.05 if self.sitl_mode else 0.2

        if is_armed:
            self.get_logger().info(f"已解鎖 -> 僅發送 Offboard 模式維持控制")
            for _ in range(5):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(sleep_time)
        else:
            self.get_logger().info(f"未解鎖 -> 執行標準起飛 (Offboard + Arm)")

            self.get_logger().info(">>> 1. 切換 Offboard 模式...")
            for _ in range(5):
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint(
                    self.current_local_ned[0], self.current_local_ned[1], self.current_local_ned[2])
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(sleep_time)

            arm_sleep = 0.2 if self.sitl_mode else 0.5
            self.get_logger().info(">>> 2. 執行解鎖 (Arm)...")
            for _ in range(5):
                self.publish_offboard_control_mode()
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                time.sleep(arm_sleep)

            self.get_logger().info("起飛指令發送完畢")

    # ═══════════════════════════════════════════════════════════════════════
    #  PX4 Message Publishers
    # ═══════════════════════════════════════════════════════════════════════

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_ctrl_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        relay_states = ('RELAY_TAKEOFF', 'RELAY_GOTO_HANDOFF', 'RELAY_EXECUTING',
                        'RELAY_INTERRUPTED', 'RELAY_HANDOFF_PENDING', 'RELAY_RETURNING')

        if self.relay_state in relay_states:
            # Setpoint leash: keep setpoint at most leash_distance ahead
            # PX4 tracks a nearby setpoint → effective speed ≈ cruise_speed
            # Using 2.0s lookahead so PX4 deadband doesn't eat the delta
            leash_distance = self.cruise_speed * 2.0  # metres ahead of current pos
            cx, cy, _ = self.current_local_ned
            dx, dy = x - cx, y - cy
            hdist = math.sqrt(dx*dx + dy*dy)
            if hdist > leash_distance and hdist > 0.01:
                scale = leash_distance / hdist
                px = cx + dx * scale
                py = cy + dy * scale
            else:
                px, py = x, y
            msg.position = [float(px), float(py), float(z)]  # z goes directly
        else:
            # 全部三軸都用 origin_ref_ned 偏移（修正 z 漂移導致起飛高度錯誤）
            msg.position = [
                float(self.origin_ref_ned[0] + x),
                float(self.origin_ref_ned[1] + y),
                float(self.origin_ref_ned[2] + z)]
        msg.yaw = self.target_yaw_ned
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = self.px4_system_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    #  Circle Mission (existing functionality, unchanged)
    # ═══════════════════════════════════════════════════════════════════════

    def circle_goal_cb(self, goal_request):
        if goal_request.commander_id != self.drone_id:
            return GoalResponse.REJECT
        if goal_request.target_id == self.drone_id:
            return GoalResponse.REJECT
        if self.circle_state != 'IDLE':
            return GoalResponse.REJECT
        self.get_logger().info(
            f"[{self.target_ns}] Accepted circle: commander={goal_request.commander_id} target={goal_request.target_id}")
        return GoalResponse.ACCEPT

    def circle_cancel_cb(self, goal_handle):
        self.circle_cancel_requested = True
        return CancelResponse.ACCEPT

    def circle_execute_cb(self, goal_handle):
        self.circle_cancel_requested = False
        target_id = goal_handle.request.target_id
        diameter = goal_handle.request.diameter if goal_handle.request.diameter > 0.0 else 3.0
        omega = goal_handle.request.omega if goal_handle.request.omega > 0.0 else 0.5

        self._start_circle(target_id, diameter, omega)
        start_time = time.time()
        feedback_msg = CircleMission.Feedback()

        while rclpy.ok():
            if self.circle_cancel_requested:
                self._stop_circle()
                goal_handle.canceled()
                result = CircleMission.Result()
                result.success = False
                result.message = "Cancelled"
                result.total_time_sec = float(time.time() - start_time)
                return result
            if self.circle_state == 'DONE':
                break
            feedback_msg.phase = self.circle_state
            feedback_msg.elapsed_sec = float(time.time() - start_time)
            if self.circle_state == 'CIRCLING' and self.circle_phase_start_time:
                dt = time.time() - self.circle_phase_start_time
                feedback_msg.angle_rad = float((self.circle_omega * dt) % (2.0 * math.pi))
            else:
                feedback_msg.angle_rad = 0.0
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        self._stop_circle()
        goal_handle.succeed()
        result = CircleMission.Result()
        result.success = True
        result.message = "Completed"
        result.total_time_sec = float(time.time() - start_time)
        return result

    def _circle_target_pos_cb(self, msg: VehicleLocalPosition):
        try:
            self.circle_target_local_pos = [float(msg.x), float(msg.y), float(msg.z)]
        except Exception:
            pass

    def _start_circle(self, target_id, diameter, omega):
        self.circle_target_id = target_id
        self.circle_radius = diameter / 2.0
        self.circle_omega = omega
        self.circle_state = 'HOVERING'
        self.circle_start_time = time.time()
        self.circle_phase_start_time = time.time()
        self.circle_target_local_pos = None

        target_ns = f'px4_{target_id}'
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.circle_target_pos_sub = self.create_subscription(
            VehicleLocalPosition, f'/{target_ns}/fmu/out/vehicle_local_position',
            self._circle_target_pos_cb, self.px4_qos)
        self.circle_target_offboard_pub = self.create_publisher(
            OffboardControlMode, f'/{target_ns}/fmu/in/offboard_control_mode', self.px4_qos)
        self.circle_target_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'/{target_ns}/fmu/in/trajectory_setpoint', self.px4_qos)
        self.circle_target_ext_ctrl_pub = self.create_publisher(
            Bool, f'/{target_ns}/external_control', cmd_qos)

        ext_msg = Bool(); ext_msg.data = True
        self.circle_target_ext_ctrl_pub.publish(ext_msg)

        wait_start = time.time()
        while self.circle_target_local_pos is None and (time.time() - wait_start) < 3.0:
            time.sleep(0.05)

        if self.circle_target_local_pos:
            self.circle_center_ned = list(self.circle_target_local_pos)
        else:
            self.circle_center_ned = [0.0, 0.0, -5.0]

        ref_msg = String()
        ref_msg.data = json.dumps({
            "active": True, "center_ned": self.circle_center_ned,
            "radius": self.circle_radius, "target_id": target_id})
        self.circle_ref_pub.publish(ref_msg)

        self.circle_timer = self.create_timer(0.05, self._circle_timer_cb)

    def _circle_timer_cb(self):
        if self.circle_state in ('IDLE', 'DONE'):
            return

        phase_elapsed = time.time() - self.circle_phase_start_time
        xc, yc, zc = self.circle_center_ned
        R = self.circle_radius

        ocm = OffboardControlMode()
        ocm.position = True; ocm.velocity = False; ocm.acceleration = False
        ocm.attitude = False; ocm.body_rate = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if self.circle_target_offboard_pub:
            self.circle_target_offboard_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.circle_state == 'HOVERING':
            sp.position = [float(xc), float(yc), float(zc)]
            sp.yaw = float('nan')
            if phase_elapsed >= 2.0:
                self.circle_state = 'MOVING_TO_EDGE'
                self.circle_phase_start_time = time.time()
        elif self.circle_state == 'MOVING_TO_EDGE':
            t_frac = min(1.0, phase_elapsed / 3.0)
            sp.position = [float(xc + R * t_frac), float(yc), float(zc)]
            sp.yaw = float('nan')
            if phase_elapsed >= 3.0:
                self.circle_state = 'CIRCLING'
                self.circle_phase_start_time = time.time()
        elif self.circle_state == 'CIRCLING':
            angle = self.circle_omega * phase_elapsed
            px = xc + R * math.cos(angle)
            py = yc + R * math.sin(angle)
            sp.position = [float(px), float(py), float(zc)]
            sp.yaw = float(math.atan2(yc - py, xc - px))

        if self.circle_target_setpoint_pub:
            self.circle_target_setpoint_pub.publish(sp)

    def _stop_circle(self):
        if self.circle_target_local_pos and self.circle_target_setpoint_pub:
            sp = TrajectorySetpoint()
            sp.position = [float(v) for v in self.circle_target_local_pos]
            sp.yaw = float('nan')
            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            ocm = OffboardControlMode()
            ocm.position = True; ocm.velocity = False; ocm.acceleration = False
            ocm.attitude = False; ocm.body_rate = False
            ocm.timestamp = sp.timestamp
            for _ in range(5):
                self.circle_target_offboard_pub.publish(ocm)
                self.circle_target_setpoint_pub.publish(sp)
                time.sleep(0.05)

        if self.circle_target_ext_ctrl_pub:
            ext_msg = Bool(); ext_msg.data = False
            self.circle_target_ext_ctrl_pub.publish(ext_msg)

        ref_msg = String()
        ref_msg.data = json.dumps({"active": False})
        self.circle_ref_pub.publish(ref_msg)

        if self.circle_timer:
            self.circle_timer.cancel()
            self.destroy_timer(self.circle_timer)
            self.circle_timer = None
        for attr in ('circle_target_pos_sub', 'circle_target_offboard_pub',
                     'circle_target_setpoint_pub', 'circle_target_ext_ctrl_pub'):
            obj = getattr(self, attr, None)
            if obj:
                if 'sub' in attr:
                    self.destroy_subscription(obj)
                else:
                    self.destroy_publisher(obj)
                setattr(self, attr, None)

        self.circle_state = 'IDLE'
        self.circle_cancel_requested = False
        self.circle_target_id = None
        self.circle_center_ned = None
        self.circle_target_local_pos = None


def main(args=None):
    rclpy.init(args=args)
    node = DroneNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
