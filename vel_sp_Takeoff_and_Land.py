import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.duration import Duration

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')
        # Thread-safe shared variables
        self.lock = threading.Lock()
        self.LandSent = False
        self.HoverSent = False
        self.TravelSent = False
        self.check = False
        
        # MAVROS connections
        self.current_state = State()
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.vel_pub = self.create_publisher(Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        # Initialize velocity command
        self.vel_cmd = Twist()
        self.last_req = self.get_clock().now()
        self.dt = 0.01

        self.create_timer(1.0, self.state_transition_callback)  
        self.create_timer(0.05, self.velocity_update_callback)  
        self.create_timer(1.0, self.landing_check_callback)     

    def state_cb(self, msg):
        with self.lock:
            self.current_state = msg
        self.get_logger().info(f"Mode: {self.current_state.mode}")
        self.get_logger().info(f"x: {self.vel_cmd.linear.x}")
        self.get_logger().info(f"y: {self.vel_cmd.linear.y}")
        self.get_logger().info(f"z: {self.vel_cmd.linear.z}")
    def response_callback(self, future):
        try:
            response = future.result()
            with self.lock:
                self.check = True
            self.get_logger().info(f'Service success: {response}')
        except Exception as e:
            self.get_logger().error(f'Service failed: {str(e)}')
    def state_transition_callback(self):
        with self.lock:
            now = self.get_clock().now()
            time_since_last_req = now - self.last_req
            current_mode = self.current_state.mode
            armed = self.current_state.armed

        if current_mode != 'OFFBOARD' and time_since_last_req > Duration(seconds=5) and not self.LandSent:
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.response_callback)
            with self.lock:
                self.last_req = now
            self.get_logger().info("Attempting OFFBOARD transition")

        elif not armed and time_since_last_req > Duration(seconds=5):
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.response_callback)
            with self.lock:
                self.last_req = now
            self.get_logger().info("Sending arm command")

    def velocity_update_callback(self):
        with self.lock:
            current_mode = self.current_state.mode
            now = self.get_clock().now()
            time_since_last_req = now - self.last_req
            hover_sent = self.HoverSent
            check_flag = self.check

        if current_mode == 'OFFBOARD':
            if time_since_last_req > Duration(seconds=12) and not hover_sent:
                with self.lock:
                    self.HoverSent = True
                    self.vel_cmd.linear.z = 0.0
                    self.vel_cmd.linear.y = 0.0
                    self.vel_cmd.linear.x = 0.0


        # Publish velocity command
        if not self.LandSent:
            with self.lock:
                self.vel_pub.publish(self.vel_cmd)

        # Handle ascent
        if not hover_sent and check_flag:
            with self.lock:
                self.vel_cmd.linear.z = min(self.vel_cmd.linear.z + self.dt, 1.0)

    def landing_check_callback(self):
        with self.lock:
            current_mode = self.current_state.mode
            now = self.get_clock().now()
            time_since_last_req = now - self.last_req

        if current_mode == 'OFFBOARD' and time_since_last_req > Duration(seconds=22):
            self.get_logger().info("Initiating landing sequence")
            with self.lock:
                self.vel_cmd.linear.x = 0.0
                self.LandSent = True
            self.land_drone()

    def land_drone(self):
        with self.lock:
            if self.current_state.mode != 'AUTO.LAND':
                req = SetMode.Request()
                req.custom_mode = 'AUTO.LAND'
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self.response_callback)
                self.get_logger().info("Landing command sent")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()