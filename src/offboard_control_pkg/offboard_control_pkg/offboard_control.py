import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.state = State()
        self.pose_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.timer = self.create_timer(0.1, self.publish_target_position)
        self.target_position = PoseStamped()
        self.target_position.pose.position.z = 3.0
        self.offboard_setpoint_counter = 0

    def state_cb(self, msg):
        self.state = msg

    def publish_target_position(self):
        if self.offboard_setpoint_counter < 100:
            self.pose_pub.publish(self.target_position)
            self.offboard_setpoint_counter += 1
        else:
            if self.state.mode != "OFFBOARD":
                self.set_mode_client.call_async(SetMode.Request(custom_mode="OFFBOARD"))
            elif not self.state.armed:
                self.arming_client.call_async(CommandBool.Request(value=True))
            self.pose_pub.publish(self.target_position)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
