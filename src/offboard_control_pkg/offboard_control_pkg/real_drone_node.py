#!/usr/bin/env python3
"""
real_drone_node.py - 針對真實 PX4 v1.14.3 無人機的最終修正版本

修正重點：
1. 加入 time.sleep 防止指令發送太快導致通訊堵塞 (解決影片中轉一下就停的問題)。
2. 修改起飛判斷邏輯：直接檢查 arming_state，解決氣壓計漂移導致無法起飛的問題。
3. QoS 設定維持 VOLATILE 以匹配真實機。
4. 新增 CircleMission Action Server：協作畫圓功能。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Point
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleGlobalPosition, VehicleLocalPosition
from mission_interfaces.action import CircleMission
import math
import json
import time  # <--- 必要：用於指令延遲

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        self.declare_parameter('drone_id', 1)
        self.declare_parameter('require_gps', True)
        self.declare_parameter('gps_eph_threshold', 2.5) # 放寬一點精度要求

        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self.require_gps = self.get_parameter('require_gps').get_parameter_value().bool_value
        self.gps_eph_threshold = self.get_parameter('gps_eph_threshold').get_parameter_value().double_value
        self.target_ns = f'px4_{self.drone_id}'

        self.get_logger().info(f"[{self.target_ns}] === 真實機 Offboard 控制版 (最終修正) ===")
        self.get_logger().info(f"[{self.target_ns}] Drone ID: {self.drone_id}, Require GPS: {self.require_gps}")

        # QoS 設定 (適配真實機 PX4)
        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.offboard_ctrl_pub = self.create_publisher(OffboardControlMode, f'/{self.target_ns}/fmu/in/offboard_control_mode', self.px4_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, f'/{self.target_ns}/fmu/in/trajectory_setpoint', self.px4_qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'/{self.target_ns}/fmu/in/vehicle_command', self.px4_qos)

        # Subscribers
        self.status_sub = self.create_subscription(VehicleStatus, f'/{self.target_ns}/fmu/out/vehicle_status', self.status_cb, self.px4_qos)
        self.global_pos_sub = self.create_subscription(VehicleGlobalPosition, f'/{self.target_ns}/fmu/out/vehicle_global_position', self.global_pos_cb, self.px4_qos)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, f'/{self.target_ns}/fmu/out/vehicle_local_position', self.local_pos_cb, self.px4_qos)

        # Laptop Commands
        self.create_subscription(String, f'/{self.target_ns}/laptop/action', self.action_cb, cmd_qos)
        self.create_subscription(Point, f'/{self.target_ns}/laptop/setpoint', self.setpoint_cb, cmd_qos)
        self.create_subscription(String, f'/{self.target_ns}/laptop/scan_action', self.scan_action_cb, cmd_qos)
        self.create_subscription(Float64, f'/{self.target_ns}/laptop/yaw_setpoint', self.yaw_setpoint_cb, cmd_qos)

        # External control gate (for being a Target)
        self.external_control_active = False
        self.create_subscription(Bool, f'/{self.target_ns}/external_control', self.external_control_cb, cmd_qos)

        # Scan Components
        self.scan_control_pub = self.create_publisher(Bool, f'/{self.target_ns}/scan_control', cmd_qos)
        self.scan_ready_pub = self.create_publisher(String, f'/{self.target_ns}/scan_ready', cmd_qos)
        self.link_quality_sub = self.create_subscription(String, f'/{self.target_ns}/link_quality', self.link_quality_cb, cmd_qos)

        # Variables
        self.target_pos_ned = [0.0, 0.0, 0.0]
        self.target_yaw_ned = float('nan')
        self.origin_ref_ned = [0.0, 0.0, 0.0]
        self.current_local_ned = [0.0, 0.0, 0.0]

        self.is_connected = False
        self.pre_flight_checks_pass = False
        self.arming_state = 0
        self.px4_system_id = 1  # 預設值，從 vehicle_status 自動更新
        self.got_global_pos = False
        self.current_eph = 999.9

        self.scan_results = {}
        self.expected_trackers = ["!e2e5b7c4", "!e2e5b8f8"]
        self.scan_completed = False

        self.status_check_count = 0

        # --- Circle Mission Action Server ---
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

        # Dynamic pub/sub for circle (created when circle starts, destroyed when ends)
        self.circle_target_pos_sub = None
        self.circle_target_offboard_pub = None
        self.circle_target_setpoint_pub = None
        self.circle_target_ext_ctrl_pub = None
        self.circle_ref_pub = self.create_publisher(String, f'/{self.target_ns}/circle_ref', cmd_qos)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"[{self.target_ns}] 等待 PX4 連線...")

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
        if self.status_check_count % 40 == 0: # 降低 log 頻率
            armed_str = "ARMED" if self.arming_state == 2 else "DISARMED"
            checks_str = "PASS" if self.pre_flight_checks_pass else "FAIL"
            self.get_logger().info(f"[{self.target_ns}] 狀態: {armed_str} | Preflight: {checks_str}")

    def global_pos_cb(self, msg):
        self.got_global_pos = True
        self.current_eph = msg.eph

    def local_pos_cb(self, msg: VehicleLocalPosition):
        try:
            self.current_local_ned = [float(msg.x), float(msg.y), float(msg.z)]
        except Exception:
            pass

    def yaw_setpoint_cb(self, msg):
        self.target_yaw_ned = float(msg.data)

    def setpoint_cb(self, msg):
        # ENU -> NED
        self.target_pos_ned = [msg.y, msg.x, -msg.z]

    def action_cb(self, msg):
        cmd = msg.data.upper()
        if cmd == "TAKEOFF_CHECK":
            self.perform_safety_takeoff()
        elif cmd == "LAND":
            self.get_logger().info(f"[{self.target_ns}] 執行降落...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif cmd == "DISARM":
            self.get_logger().info(f"[{self.target_ns}] 強制上鎖...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        elif cmd == "RESET_ORIGIN":
            self.origin_ref_ned = list(self.current_local_ned)
            self.target_yaw_ned = float('nan')
            self.get_logger().info(f"[{self.target_ns}] 重置相對原點 (PX4 內部原點未變)")

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
        """External control gate: True = pause own 20Hz publishing, False = resume."""
        prev = self.external_control_active
        self.external_control_active = msg.data
        if msg.data and not prev:
            self.get_logger().info(f"[{self.target_ns}] External control ACTIVE - pausing own setpoint publishing")
        elif not msg.data and prev:
            self.get_logger().info(f"[{self.target_ns}] External control RELEASED - resuming own setpoint publishing")

    # --- Circle Mission Action callbacks ---
    def circle_goal_cb(self, goal_request):
        """Validate incoming circle mission goal."""
        if goal_request.commander_id != self.drone_id:
            self.get_logger().warn(f"[{self.target_ns}] Rejected: commander_id={goal_request.commander_id} != my id={self.drone_id}")
            return GoalResponse.REJECT
        if goal_request.target_id == self.drone_id:
            self.get_logger().warn(f"[{self.target_ns}] Rejected: target cannot be self")
            return GoalResponse.REJECT
        if self.circle_state != 'IDLE':
            self.get_logger().warn(f"[{self.target_ns}] Rejected: circle mission already active (state={self.circle_state})")
            return GoalResponse.REJECT
        self.get_logger().info(f"[{self.target_ns}] Accepted circle mission: commander={goal_request.commander_id} target={goal_request.target_id}")
        return GoalResponse.ACCEPT

    def circle_cancel_cb(self, goal_handle):
        """Accept cancel request."""
        self.get_logger().info(f"[{self.target_ns}] Circle mission cancel requested")
        self.circle_cancel_requested = True
        return CancelResponse.ACCEPT

    def circle_execute_cb(self, goal_handle):
        """Main execute callback for circle mission (runs in action callback group thread)."""
        self.get_logger().info(f"[{self.target_ns}] Circle mission executing...")
        self.circle_cancel_requested = False

        target_id = goal_handle.request.target_id
        diameter = goal_handle.request.diameter if goal_handle.request.diameter > 0.0 else 3.0
        omega = goal_handle.request.omega if goal_handle.request.omega > 0.0 else 0.5

        # Start the circle
        self._start_circle(target_id, diameter, omega)

        start_time = time.time()
        feedback_msg = CircleMission.Feedback()

        # Feedback loop at ~10Hz
        while rclpy.ok():
            if self.circle_cancel_requested:
                self.get_logger().info(f"[{self.target_ns}] Circle mission cancelled")
                self._stop_circle()
                goal_handle.canceled()
                result = CircleMission.Result()
                result.success = False
                result.message = "Cancelled by user"
                result.total_time_sec = float(time.time() - start_time)
                return result

            if self.circle_state == 'DONE':
                break

            # Send feedback
            feedback_msg.phase = self.circle_state
            feedback_msg.elapsed_sec = float(time.time() - start_time)
            if self.circle_state == 'CIRCLING' and self.circle_phase_start_time is not None:
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
        result.message = "Circle mission completed"
        result.total_time_sec = float(time.time() - start_time)
        return result

    def _circle_target_pos_cb(self, msg: VehicleLocalPosition):
        """Callback for target drone's VehicleLocalPosition."""
        try:
            self.circle_target_local_pos = [float(msg.x), float(msg.y), float(msg.z)]
        except Exception:
            pass

    def _start_circle(self, target_id, diameter, omega):
        """Initialize circle mission: create dynamic pub/sub, send external_control, start timer."""
        self.circle_target_id = target_id
        self.circle_radius = diameter / 2.0
        self.circle_omega = omega
        self.circle_state = 'HOVERING'
        self.circle_start_time = time.time()
        self.circle_phase_start_time = time.time()
        self.circle_target_local_pos = None

        target_ns = f'px4_{target_id}'
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Create dynamic subscribers/publishers for the target drone
        self.circle_target_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/{target_ns}/fmu/out/vehicle_local_position',
            self._circle_target_pos_cb,
            self.px4_qos)

        self.circle_target_offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'/{target_ns}/fmu/in/offboard_control_mode',
            self.px4_qos)

        self.circle_target_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/{target_ns}/fmu/in/trajectory_setpoint',
            self.px4_qos)

        self.circle_target_ext_ctrl_pub = self.create_publisher(
            Bool, f'/{target_ns}/external_control', cmd_qos)

        # Tell target to pause its own publishing
        ext_msg = Bool()
        ext_msg.data = True
        self.circle_target_ext_ctrl_pub.publish(ext_msg)

        # Wait for target position data
        self.get_logger().info(f"[{self.target_ns}] Waiting for target drone {target_id} position...")
        wait_start = time.time()
        while self.circle_target_local_pos is None and (time.time() - wait_start) < 3.0:
            time.sleep(0.05)

        if self.circle_target_local_pos is not None:
            self.circle_center_ned = list(self.circle_target_local_pos)
            self.get_logger().info(
                f"[{self.target_ns}] Target position (circle center): "
                f"[{self.circle_center_ned[0]:.2f}, {self.circle_center_ned[1]:.2f}, {self.circle_center_ned[2]:.2f}]")
        else:
            self.get_logger().warn(f"[{self.target_ns}] No target position received, using [0,0,-5]")
            self.circle_center_ned = [0.0, 0.0, -5.0]

        # Publish circle reference for visualizer
        ref_data = json.dumps({
            "active": True,
            "center_ned": self.circle_center_ned,
            "radius": self.circle_radius,
            "target_id": target_id
        })
        ref_msg = String()
        ref_msg.data = ref_data
        self.circle_ref_pub.publish(ref_msg)

        # Start 20Hz circle control timer
        self.circle_timer = self.create_timer(0.05, self._circle_timer_cb)
        self.get_logger().info(f"[{self.target_ns}] Circle timer started (R={self.circle_radius:.1f}m, omega={self.circle_omega:.2f} rad/s)")

    def _circle_timer_cb(self):
        """20Hz state machine for circle mission, publishes setpoints to target's PX4."""
        if self.circle_state == 'IDLE' or self.circle_state == 'DONE':
            return

        now = time.time()
        phase_elapsed = now - self.circle_phase_start_time
        xc, yc, zc = self.circle_center_ned
        R = self.circle_radius

        # Publish OffboardControlMode to target
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if self.circle_target_offboard_pub:
            self.circle_target_offboard_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.circle_state == 'HOVERING':
            # Hover at circle center for 2 seconds
            sp.position = [float(xc), float(yc), float(zc)]
            sp.yaw = float('nan')
            if phase_elapsed >= 2.0:
                self.circle_state = 'MOVING_TO_EDGE'
                self.circle_phase_start_time = time.time()
                self.get_logger().info(f"[{self.target_ns}] Circle phase: MOVING_TO_EDGE")

        elif self.circle_state == 'MOVING_TO_EDGE':
            # Linearly interpolate from center to edge (xc+R, yc, zc) over 3 seconds
            t_frac = min(1.0, phase_elapsed / 3.0)
            sp.position = [float(xc + R * t_frac), float(yc), float(zc)]
            sp.yaw = float('nan')
            if phase_elapsed >= 3.0:
                self.circle_state = 'CIRCLING'
                self.circle_phase_start_time = time.time()
                self.get_logger().info(f"[{self.target_ns}] Circle phase: CIRCLING")

        elif self.circle_state == 'CIRCLING':
            # Circular trajectory: x = xc + R*cos(wt), y = yc + R*sin(wt)
            dt = phase_elapsed
            angle = self.circle_omega * dt
            px = xc + R * math.cos(angle)
            py = yc + R * math.sin(angle)
            sp.position = [float(px), float(py), float(zc)]
            # Yaw: face toward circle center
            sp.yaw = float(math.atan2(yc - py, xc - px))

        if self.circle_target_setpoint_pub:
            self.circle_target_setpoint_pub.publish(sp)

    def _stop_circle(self):
        """Stop circle mission: send last setpoint, release external control, cleanup."""
        self.get_logger().info(f"[{self.target_ns}] Stopping circle mission...")

        # Send current target position as final setpoint (hold position)
        if self.circle_target_local_pos and self.circle_target_setpoint_pub:
            sp = TrajectorySetpoint()
            sp.position = [float(self.circle_target_local_pos[0]),
                           float(self.circle_target_local_pos[1]),
                           float(self.circle_target_local_pos[2])]
            sp.yaw = float('nan')
            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            ocm = OffboardControlMode()
            ocm.position = True
            ocm.velocity = False
            ocm.acceleration = False
            ocm.attitude = False
            ocm.body_rate = False
            ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            # Send a few times to ensure delivery
            for _ in range(5):
                self.circle_target_offboard_pub.publish(ocm)
                self.circle_target_setpoint_pub.publish(sp)
                time.sleep(0.05)

        # Release external control on target
        if self.circle_target_ext_ctrl_pub:
            ext_msg = Bool()
            ext_msg.data = False
            self.circle_target_ext_ctrl_pub.publish(ext_msg)

        # Publish circle_ref inactive for visualizer
        ref_msg = String()
        ref_msg.data = json.dumps({"active": False})
        self.circle_ref_pub.publish(ref_msg)

        # Stop and destroy circle timer
        if self.circle_timer is not None:
            self.circle_timer.cancel()
            self.destroy_timer(self.circle_timer)
            self.circle_timer = None

        # Destroy dynamic pub/sub
        if self.circle_target_pos_sub is not None:
            self.destroy_subscription(self.circle_target_pos_sub)
            self.circle_target_pos_sub = None
        if self.circle_target_offboard_pub is not None:
            self.destroy_publisher(self.circle_target_offboard_pub)
            self.circle_target_offboard_pub = None
        if self.circle_target_setpoint_pub is not None:
            self.destroy_publisher(self.circle_target_setpoint_pub)
            self.circle_target_setpoint_pub = None
        if self.circle_target_ext_ctrl_pub is not None:
            self.destroy_publisher(self.circle_target_ext_ctrl_pub)
            self.circle_target_ext_ctrl_pub = None

        self.circle_state = 'IDLE'
        self.circle_cancel_requested = False
        self.circle_target_id = None
        self.circle_center_ned = None
        self.circle_target_local_pos = None
        self.get_logger().info(f"[{self.target_ns}] Circle mission stopped, state=IDLE")

    def perform_safety_takeoff(self):
        """修正後的起飛流程"""
        self.get_logger().info(f"[{self.target_ns}] ===== 起飛檢查 =====")

        if not self.is_connected:
            self.get_logger().error("未連接 PX4")
            return
        if not self.pre_flight_checks_pass:
            self.get_logger().error("Preflight Checks 失敗 (請檢查 QGC)")
            return

        if self.require_gps:
            if not self.got_global_pos or self.current_eph > self.gps_eph_threshold:
                self.get_logger().error(f"GPS 精度不足: {self.current_eph:.2f}m")
                return

        # --- 核心修正：使用 arming_state 判斷 ---
        is_armed = (self.arming_state == 2)

        if is_armed:
            self.get_logger().info(f"已解鎖 -> 僅發送 Offboard 模式維持控制")
            for _ in range(5):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(0.1)
        else:
            self.get_logger().info(f"未解鎖 -> 執行標準起飛 (Offboard + Arm)")
            self.get_logger().info(f"請確認遙控器油門已拉到最低！")

            # 1. 設定目標高度
            self.target_pos_ned = [0.0, 0.0, -5.0]

            # 2. 先切換模式 (給予緩衝時間)
            self.get_logger().info(">>> 1. 切換 Offboard 模式...")
            for i in range(5):
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint(0.0, 0.0, 0.0) # 先送 0,0,0 防止暴衝
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                time.sleep(0.2) # 每個指令間隔 0.2 秒

            # 3. 再執行解鎖
            self.get_logger().info(">>> 2. 執行解鎖 (Arm)...")
            for i in range(5):
                self.publish_offboard_control_mode() # 持續發送 heartbeat
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                time.sleep(0.5) # 解鎖需要較長時間確認

            self.get_logger().info("起飛指令發送完畢")

    def timer_callback(self):
        if not self.external_control_active:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(self.target_pos_ned[0], self.target_pos_ned[1], self.target_pos_ned[2])

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
        # 加上原點偏移量
        msg.position = [
            float(self.origin_ref_ned[0] + x),
            float(self.origin_ref_ned[1] + y),
            float(z)
        ]
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
