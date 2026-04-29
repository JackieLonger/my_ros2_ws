#!/usr/bin/env python3
"""
multi_drone_sender.py — 乾淨測試版（手動模式 + 接力模式）

主選單：
  T  起飛
  G  接力任務（需 -p mission_file）
  Space  恢復暫停的接力（從原地繼續飛剩餘航點）
  X  中斷當前接力機（觸發交接給下一台）
  P  緊急原地懸停（暫停，不清除接力進度）
  L  降落
  R  上鎖 (Disarm)
  M  進入手動模式
  1/2/3/5 切機（5=全體）
  Ctrl+C 離開

手動模式：
  T  起飛
  W/S 前/後  A/D 左/右平移
  I/K 升/降  J/L 左/右旋轉
  Y 懸停 HOLD  +/- 步進
  P 緊急懸停（退出手動）  B 降落（退出手動）  R 上鎖（退出手動）
  M 退出手動  1/2/3/5 切機
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mission_interfaces.msg import GlobalMission, RelayStatus
import sys, termios, tty, time, select, math
import threading
import json
from datetime import datetime


class Colors:
    HEADER = '\033[95m'; GREEN = '\033[92m'; YELLOW = '\033[93m'; RED = '\033[91m'; ENDC = '\033[0m'


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        # --- Parameters ---
        self.declare_parameter('drone_ids', '1,2')
        self.declare_parameter('mission_file', '')
        ids_raw = self.get_parameter('drone_ids').get_parameter_value().string_value
        parsed_ids = []
        try:
            for token in ids_raw.split(','):
                token = token.strip()
                if token.isdigit():
                    parsed_ids.append(int(token))
        except Exception:
            parsed_ids = []
        self.ids = sorted(set(parsed_ids)) if parsed_ids else [1, 2]
        self.mission_file = self.get_parameter('mission_file').get_parameter_value().string_value

        self.target_mode = "ALL"
        self.takeoff_height = 5.0  # ENU 上升 5 m

        # --- Manual mode state ---
        self.manual_mode = False
        self.manual_position = {did: [0.0, 0.0, self.takeoff_height] for did in self.ids}
        self.manual_yaw = {did: 0.0 for did in self.ids}
        self.position_step = 0.5
        self.yaw_step = 0.2  # ~11.5°
        self.manual_bounds = {'x': (-20.0, 20.0), 'y': (-20.0, 20.0), 'z': (1.0, 15.0)}

        # --- Publishers / Subscribers ---
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.action_pubs = {}
        self.setpoint_pubs = {}
        self.yaw_pubs = {}
        for did in self.ids:
            self.action_pubs[did] = self.create_publisher(String, f'/px4_{did}/laptop/action', qos)
            self.setpoint_pubs[did] = self.create_publisher(Point, f'/px4_{did}/laptop/setpoint', qos)
            self.yaw_pubs[did] = self.create_publisher(Float64, f'/px4_{did}/laptop/yaw_setpoint', qos)

        # --- Relay mission ---
        self.global_mission_pub = self.create_publisher(GlobalMission, '/swarm/global_mission', qos)
        self.planned_gps_pub = self.create_publisher(NavSatFix, '/swarm/planned_gps', qos)
        self.relay_interrupt_pubs = {
            did: self.create_publisher(Bool, f'/px4_{did}/relay_interrupt', qos)
            for did in self.ids
        }
        self.create_subscription(String, '/swarm/map_received', self.map_received_cb, qos)
        self.create_subscription(RelayStatus, '/swarm/relay_status', self.relay_status_cb, qos)

        self.mission_waypoints = []  # [(lat, lon, alt), ...]
        self.map_ack_count = 0
        self.relay_active_drone = 0
        self.relay_started_drone = 0
        self.relay_phase = "IDLE"
        self.relay_current_index = 0
        self.relay_total = 0

        if self.mission_file:
            self._load_plan_file(self.mission_file)

        # Background spin thread
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        self.print_ui()

    def _spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    # ────────────────────────────────────────────────────────────
    # UI
    # ────────────────────────────────────────────────────────────
    def print_ui(self):
        print("\033c", end="")
        target_str = "【全體 All】" if self.target_mode == "ALL" else f"【單機 ID: {self.target_mode}】"
        print(f"{Colors.HEADER}=== 測試控制台（手動 + 接力） ==={Colors.ENDC}")
        print(f"當前控制: {Colors.GREEN}{target_str}{Colors.ENDC}")
        print(f"--------------------------------------------")
        print(f"[T]      起飛")
        print(f"{Colors.GREEN}[G]      接力任務 (Relay: 廣播地圖+啟動){Colors.ENDC}")
        print(f"{Colors.GREEN}[Space]  恢復暫停的接力（從原地繼續）{Colors.ENDC}")
        print(f"{Colors.RED}[X]      中斷接力 (觸發交接給下一台){Colors.ENDC}")
        print(f"{Colors.RED}[P]      原地懸停（暫停，不清進度）{Colors.ENDC}")
        print(f"{Colors.YELLOW}[H]      返航 (Return Home){Colors.ENDC}")
        print(f"[L]      降落 (Land)")
        print(f"[R]      上鎖 (Disarm) - 降落後必按")
        print(f"{Colors.YELLOW}[M]      手動鍵盤控制 (Manual Mode){Colors.ENDC}")
        print(f"--------------------------------------------")
        if self.mission_waypoints:
            print(f"地圖: {len(self.mission_waypoints)} 航點已載入")
        if self.relay_active_drone > 0:
            print(f"接力: Drone {self.relay_active_drone} | {self.relay_phase} | "
                  f"{self.relay_current_index}/{self.relay_total}")
        print(f"--------------------------------------------")
        ids_hint = "/".join(str(d) for d in self.ids)
        print(f"[{ids_hint}/5] 切換飛機 (5=全體)")
        print(f"[Ctrl+C] 離開")

    def set_target(self, mode):
        self.target_mode = mode
        if not self.manual_mode:
            self.print_ui()

    # ────────────────────────────────────────────────────────────
    # 共用發送
    # ────────────────────────────────────────────────────────────
    def send_action(self, action):
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        msg = String(); msg.data = action
        for did in targets:
            self.action_pubs[did].publish(msg)

    def send_setpoint(self, x, y, z):
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        msg = Point(); msg.x = float(x); msg.y = float(y); msg.z = float(z)
        for did in targets:
            self.setpoint_pubs[did].publish(msg)

    def send_setpoint_per_drone(self, did, x, y, z):
        msg = Point(); msg.x = float(x); msg.y = float(y); msg.z = float(z)
        self.setpoint_pubs[did].publish(msg)

    def send_yaw_per_drone(self, did, yaw_rad):
        msg = Float64(); msg.data = float(yaw_rad)
        self.yaw_pubs[did].publish(msg)

    # ────────────────────────────────────────────────────────────
    # 接力任務
    # ────────────────────────────────────────────────────────────
    def _load_plan_file(self, filepath):
        """Parse QGC .plan file, extract MAV_CMD_NAV_WAYPOINT (cmd=16)."""
        try:
            with open(filepath, 'r') as f:
                plan = json.load(f)
            items = plan.get('mission', {}).get('items', [])
            self.mission_waypoints = []
            for item in items:
                if item.get('command', 0) == 16:
                    params = item.get('params', [])
                    if len(params) >= 7 and params[4] is not None and params[5] is not None:
                        lat, lon, alt = float(params[4]), float(params[5]), float(params[6] or 0)
                        self.mission_waypoints.append((lat, lon, alt))
            print(f"{Colors.GREEN}>>> .plan 載入成功: {len(self.mission_waypoints)} 個航點{Colors.ENDC}")
        except Exception as e:
            print(f"{Colors.RED}>>> .plan 載入失敗: {e}{Colors.ENDC}")

    def broadcast_global_mission(self):
        if not self.mission_waypoints:
            print(f"{Colors.RED}>>> 無航點可廣播，請先載入 .plan 檔{Colors.ENDC}")
            return
        msg = GlobalMission()
        msg.mission_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        msg.waypoint_lats = [wp[0] for wp in self.mission_waypoints]
        msg.waypoint_lons = [wp[1] for wp in self.mission_waypoints]
        msg.waypoint_alts = [wp[2] for wp in self.mission_waypoints]
        msg.mission_altitude = 10.0
        self.map_ack_count = 0
        for _ in range(5):
            self.global_mission_pub.publish(msg)
            time.sleep(0.1)
        print(f"{Colors.GREEN}>>> GlobalMission 已廣播: {len(self.mission_waypoints)} 航點{Colors.ENDC}")

        for lat, lon, alt in self.mission_waypoints:
            nav = NavSatFix()
            nav.latitude = lat; nav.longitude = lon; nav.altitude = alt
            self.planned_gps_pub.publish(nav)

        print("等待無人機確認地圖接收...")
        wait_start = time.time()
        while self.map_ack_count < len(self.ids) and (time.time() - wait_start) < 10.0:
            time.sleep(0.5)
        if self.map_ack_count >= len(self.ids):
            print(f"{Colors.GREEN}>>> 全部 {self.map_ack_count} 台已確認{Colors.ENDC}")
        else:
            print(f"{Colors.YELLOW}>>> {self.map_ack_count}/{len(self.ids)} 台確認（部分超時）{Colors.ENDC}")

    def map_received_cb(self, msg):
        self.map_ack_count += 1

    def relay_status_cb(self, msg: RelayStatus):
        if msg.phase in ('IDLE', 'RELAY_IDLE'):
            return
        self.relay_active_drone = msg.active_drone_id
        self.relay_phase = msg.phase
        self.relay_current_index = msg.current_index
        self.relay_total = msg.total_waypoints

    def perform_relay_start(self):
        if not self.mission_waypoints:
            print(f"{Colors.RED}>>> 請先載入 .plan 檔 (啟動時 -p mission_file:=xxx){Colors.ENDC}")
            input("按 Enter 返回...")
            self.print_ui()
            return
        print(f"\n{Colors.YELLOW}>>> 接力任務啟動程序...{Colors.ENDC}")
        self.broadcast_global_mission()
        first_drone = self.ids[0]
        self.relay_started_drone = first_drone
        self.relay_active_drone = first_drone
        msg = String(); msg.data = "START_RELAY"
        self.action_pubs[first_drone].publish(msg)
        print(f"{Colors.GREEN}>>> 已下令 Drone {first_drone} 開始接力任務{Colors.ENDC}")
        print(f"按 [P] 暫停、[Space] 恢復、[X] 交接")
        time.sleep(1.0)
        self.print_ui()

    def perform_relay_interrupt(self):
        active = self.relay_active_drone if self.relay_active_drone > 0 else (
            self.relay_started_drone if self.relay_started_drone > 0 else self.ids[0])
        if active not in self.relay_interrupt_pubs:
            print(f"{Colors.RED}>>> 無法中斷: drone {active} 不在列表中{Colors.ENDC}")
            return
        msg = Bool(); msg.data = True
        self.relay_interrupt_pubs[active].publish(msg)
        print(f"\n{Colors.YELLOW}>>> 已發送中斷指令給 Drone {active}{Colors.ENDC}")
        print(f">>> Drone {active} 將自動交接給下一台")

    def perform_force_hover(self):
        """原地懸停：依 target_mode（單機或全體）發 FORCE_HOVER。
        drone 端設 paused=True，懸停在當前位置；relay_state 保留。
        可後續按 Space 恢復、X 交接、L 降落、H 返航、R 上鎖。"""
        target_str = "全體" if self.target_mode == "ALL" else f"Drone {self.target_mode}"
        print(f"\n{Colors.RED}>>> {target_str} 原地懸停（暫停） — 按 Space 恢復、X 交接、L 降落、H 返航{Colors.ENDC}")
        self.send_action("FORCE_HOVER")
        if self.manual_mode:
            self.manual_mode = False
            self.print_ui()

    def perform_return_home(self):
        """通知選定 drone 返航：飛到 return_altitude → 飛回起點上空 → 降落。"""
        target_str = "全體" if self.target_mode == "ALL" else f"Drone {self.target_mode}"
        print(f"\n{Colors.YELLOW}>>> {target_str} 返航{Colors.ENDC}")
        self.send_action("RETURN_HOME")

    def perform_relay_resume(self):
        """從暫停狀態恢復：通知所有 drone 解除 paused，state machine 從當前 relay_state 繼續推進。"""
        print(f"\n{Colors.GREEN}>>> 恢復接力任務（從暫停處繼續）{Colors.ENDC}")
        msg = String(); msg.data = "RESUME_RELAY"
        for did in self.ids:
            self.action_pubs[did].publish(msg)
        time.sleep(0.5)
        self.print_ui()

    # ────────────────────────────────────────────────────────────
    # 起飛 / 降落 / 上鎖
    # ────────────────────────────────────────────────────────────
    def perform_takeoff(self):
        print(f"\n{Colors.YELLOW}>>> 執行起飛程序...{Colors.ENDC}")
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        self.send_action("RESET_ORIGIN")
        self.send_action("TAKEOFF_CHECK")
        time.sleep(0.5)
        self.send_setpoint(0.0, 0.0, self.takeoff_height)
        # 同步手動模式內部位置（即使現在不在手動，下次進手動會用 enter_manual_mode 再覆寫）
        for did in targets:
            self.manual_position[did] = [0.0, 0.0, self.takeoff_height]
            self.manual_yaw[did] = 0.0
        print(f">>> 指令已發送，等待無人機爬升至懸停高度 ({self.takeoff_height} m)...")

    def perform_land(self):
        print(f"\n{Colors.RED}>>> 執行降落...{Colors.ENDC}")
        self.send_action("LAND")
        time.sleep(1.0)
        self.send_action("RESET_ORIGIN")
        self.send_setpoint(0.0, 0.0, 0.0)
        # 降落清除接力顯示
        self.relay_active_drone = 0
        self.relay_started_drone = 0
        self.relay_phase = "IDLE"

    def perform_disarm(self):
        print(f"\n{Colors.RED}>>> 強制上鎖 (Disarm){Colors.ENDC}")
        self.send_action("DISARM")
        print(">>> 狀態已重置，可重新執行起飛 [T]")

    # ────────────────────────────────────────────────────────────
    # 手動模式
    # ────────────────────────────────────────────────────────────
    def print_manual_ui(self):
        print("\033c", end="")
        target_str = "【全體 All】" if self.target_mode == "ALL" else f"【單機 ID: {self.target_mode}】"
        print(f"{Colors.HEADER}=== 手動鍵盤控制模式 ==={Colors.ENDC}")
        print(f"當前控制: {Colors.GREEN}{target_str}{Colors.ENDC}")
        print(f"位置步進: {Colors.YELLOW}{self.position_step:.2f} m{Colors.ENDC}  偏航步進: {Colors.YELLOW}{math.degrees(self.yaw_step):.1f}°{Colors.ENDC}")
        print(f"--------------------------------------------")
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        for did in targets:
            pos = self.manual_position[did]
            yaw_deg = math.degrees(self.manual_yaw[did])
            print(f"  Drone {did}: [{pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f}] Yaw={yaw_deg:+.1f}°")
        print(f"--------------------------------------------")
        print(f"[T]   起飛")
        print(f"[W/S] 機頭前進/後退    [A/D] 左/右平移")
        print(f"[I/K] 爬升/下降        [J/L] 機頭左/右旋轉")
        print(f"[Y]   懸停 HOLD        [+/-] 調整步進")
        print(f"{Colors.RED}[P] 原地懸停（退出手動）{Colors.ENDC}")
        print(f"{Colors.YELLOW}[H] 返航 (退出手動){Colors.ENDC}")
        print(f"[B] 降落  [R] 上鎖")
        ids_hint = "/".join(str(d) for d in self.ids)
        print(f"[{ids_hint}/5] 切換飛機")
        print(f"[M] 退出手動模式")

    def enter_manual_mode(self):
        """進手動：停接力（如有暫停先解除）、重設原點、爬到 5m 懸停、manual_position 歸零。"""
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        for did in targets:
            msg = String(); msg.data = "RESET_ORIGIN"
            self.action_pubs[did].publish(msg)
        time.sleep(0.3)
        for did in targets:
            self.send_setpoint_per_drone(did, 0.0, 0.0, self.takeoff_height)
            self.manual_position[did] = [0.0, 0.0, self.takeoff_height]
            self.manual_yaw[did] = 0.0
            self.send_yaw_per_drone(did, 0.0)
        self.print_manual_ui()

    def handle_manual_key(self, key):
        targets = self.ids if self.target_mode == "ALL" else [self.target_mode]
        body_fwd = 0.0
        body_right = 0.0
        dz = 0.0
        yaw_delta = 0.0
        step = self.position_step

        if key in ('t', 'T'):
            self.perform_takeoff()
            time.sleep(0.5)
            self.print_manual_ui()
            return
        elif key in ('w', 'W'):
            body_fwd = step
        elif key in ('s', 'S'):
            body_fwd = -step
        elif key in ('a', 'A'):
            body_right = -step
        elif key in ('d', 'D'):
            body_right = step
        elif key in ('i', 'I'):
            dz = step
        elif key in ('k', 'K'):
            dz = -step
        elif key in ('j', 'J'):
            yaw_delta = -self.yaw_step
        elif key in ('l', 'L'):
            yaw_delta = self.yaw_step
        elif key in ('y', 'Y'):
            for did in targets:
                pos = self.manual_position[did]
                self.send_setpoint_per_drone(did, pos[0], pos[1], pos[2])
                self.send_yaw_per_drone(did, self.manual_yaw[did])
            self.print_manual_ui()
            return
        elif key in ('+', '='):
            self.position_step = min(5.0, self.position_step + 0.25)
            self.yaw_step = min(math.radians(45), self.yaw_step + math.radians(5))
            self.print_manual_ui()
            return
        elif key == '-':
            self.position_step = max(0.1, self.position_step - 0.25)
            self.yaw_step = max(math.radians(5), self.yaw_step - math.radians(5))
            self.print_manual_ui()
            return
        else:
            return

        if body_fwd == 0.0 and body_right == 0.0 and dz == 0.0 and yaw_delta == 0.0:
            return

        bounds = self.manual_bounds
        for did in targets:
            pos = self.manual_position[did]
            if body_fwd != 0.0 or body_right != 0.0:
                yaw = self.manual_yaw[did]
                # ENU: X=East, Y=North. NED yaw=0 → North → ENU Y+
                dx = body_fwd * math.sin(yaw) + body_right * math.cos(yaw)
                dy = body_fwd * math.cos(yaw) - body_right * math.sin(yaw)
                pos[0] = max(bounds['x'][0], min(bounds['x'][1], pos[0] + dx))
                pos[1] = max(bounds['y'][0], min(bounds['y'][1], pos[1] + dy))
            if dz != 0.0:
                pos[2] = max(bounds['z'][0], min(bounds['z'][1], pos[2] + dz))

            self.send_setpoint_per_drone(did, pos[0], pos[1], pos[2])

            if yaw_delta != 0.0:
                self.manual_yaw[did] += yaw_delta
                self.manual_yaw[did] = math.atan2(math.sin(self.manual_yaw[did]), math.cos(self.manual_yaw[did]))
                self.send_yaw_per_drone(did, self.manual_yaw[did])

        self.print_manual_ui()


# ────────────────────────────────────────────────────────────
# Keyboard helpers
# ────────────────────────────────────────────────────────────
def _drain_stdin():
    """Consume any remaining bytes in stdin to prevent leftover escape-sequence
    bytes (e.g. '[A' after a missed arrow key) being read as standalone keys."""
    while select.select([sys.stdin], [], [], 0.0)[0]:
        try:
            sys.stdin.read(1)
        except Exception:
            break


def get_key():
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            if select.select([sys.stdin], [], [], 0.3)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    if select.select([sys.stdin], [], [], 0.3)[0]:
                        ch3 = sys.stdin.read(1)
                        arrow_map = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}
                        if ch3 in arrow_map:
                            return arrow_map[ch3]
                        _drain_stdin()
                        return None
                    _drain_stdin()
                    return None
                _drain_stdin()
                return None
            return '\x1b'
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        while rclpy.ok():
            key = get_key()
            if key is None:
                continue
            if key == '\x03':
                break

            # 數字鍵切機（兩種模式都可）
            if key.isdigit():
                did = int(key)
                if did == 5:
                    node.set_target("ALL")
                    if node.manual_mode:
                        node.print_manual_ui()
                elif did in node.ids:
                    node.set_target(did)
                    if node.manual_mode:
                        node.print_manual_ui()

            # M 切換手動
            elif key in ('m', 'M'):
                node.manual_mode = not node.manual_mode
                if node.manual_mode:
                    node.enter_manual_mode()
                else:
                    node.print_ui()

            # 手動模式
            elif node.manual_mode:
                if key in ('p', 'P'):
                    node.perform_force_hover()
                elif key in ('h', 'H'):
                    node.perform_return_home(); node.manual_mode = False; node.print_ui()
                elif key in ('b', 'B'):
                    node.perform_land(); node.manual_mode = False; node.print_ui()
                elif key in ('r', 'R'):
                    node.perform_disarm(); node.manual_mode = False; node.print_ui()
                else:
                    node.handle_manual_key(key)

            # 一般模式
            else:
                if key in ('t', 'T'): node.perform_takeoff()
                elif key in ('g', 'G'): node.perform_relay_start()
                elif key == ' ': node.perform_relay_resume()
                elif key in ('x', 'X'): node.perform_relay_interrupt()
                elif key in ('p', 'P'): node.perform_force_hover()
                elif key in ('h', 'H'): node.perform_return_home()
                elif key in ('l', 'L'): node.perform_land()
                elif key in ('r', 'R'): node.perform_disarm()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
