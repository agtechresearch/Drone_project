import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        self.current_state = State()
        self.counter = 0
        self.took_off = False
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('이륙 노드 시작!')

    def state_cb(self, msg):
        self.current_state = msg
        if msg.connected:
            self.get_logger().info('FCU 연결됨!')

    def control_loop(self):
        if not self.current_state.connected:
            return

        # 20초 후 GUIDED 모드
        if self.counter == 200:
            self.get_logger().info('GUIDED 모드 전환 중...')
            self.set_mode('GUIDED')

        # 25초 후 ARM
        if self.counter == 250:
            self.get_logger().info('ARM 중...')
            self.arm()

        # 30초 후 이륙 명령
        if self.counter == 300 and not self.took_off:
            self.get_logger().info('이륙 명령 전송!')
            self.takeoff(5.0)  # 5m 이륙
            self.took_off = True

        self.counter += 1

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)
        self.get_logger().info(f'모드 변경: {mode}')

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)
        self.get_logger().info('ARM 요청 전송')

    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = altitude
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        self.takeoff_client.call_async(req)
        self.get_logger().info(f'{altitude}m 이륙 명령 전송!')

def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
