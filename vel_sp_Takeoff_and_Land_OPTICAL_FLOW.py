import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu  # I am assuming optical flow data as IMU data
from rclpy.duration import Duration

class OpticalFlowFlightNode(Node):
    def __init__(self):
        super().__init__('optical_flow_flight_node')
        self.lock = threading.Lock()
        self.LandSent = False

        # MAVROS state and commands
        self.current_state = State()
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.vel_pub = self.create_publisher(Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        
        # Optical flow subscription
        self.flow_sub = self.create_subscription(Imu, '/mavros/px4flow/raw/optical_flow_rad', self.flow_callback, 10)
        
        self.vel_cmd = Twist()
        self.last_req = self.get_clock().now()
        self.flow_x = 0.0
        self.flow_y = 0.0

        self.create_timer(1.0, self.state_transition_callback)
        self.create_timer(0.05, self.velocity_update_callback)
        self.create_timer(1.0, self.landing_check_callback)

    def state_cb(self, msg):
        with self.lock:
            self.current_state = msg
        self.get_logger().info(f"Mode: {self.current_state.mode}")

    def flow_callback(self, msg):
        with self.lock:
            self.flow_x = msg.angular_velocity.x  # Optical flow in x direction
            self.flow_y = msg.angular_velocity.y  # Optical flow in y direction
        self.get_logger().info(f"Optical Flow - X: {self.flow_x}, Y: {self.flow_y}")

    def response_callback(self, future):
        try:
            response = future.result()
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

        if current_mode == 'OFFBOARD':
            if time_since_last_req < Duration(seconds=12):  
                with self.lock:
                    self.vel_cmd.linear.z = 1.0
            elif time_since_last_req < Duration(seconds=40):  
                with self.lock:
                    self.vel_cmd.linear.x = -self.flow_x  
                    self.vel_cmd.linear.y = -self.flow_y
                    self.vel_cmd.linear.z = 0.0  
            elif time_since_last_req >= Duration(seconds=42):  
                with self.lock:
                    self.LandSent = True
                self.land_drone()

        if not self.LandSent:
            with self.lock:
                self.vel_pub.publish(self.vel_cmd)

    def landing_check_callback(self):
        with self.lock:
            current_mode = self.current_state.mode
            now = self.get_clock().now()
            time_since_last_req = now - self.last_req

        if current_mode == 'OFFBOARD' and time_since_last_req > Duration(seconds=44):
            self.get_logger().info("Initiating landing sequence")
            with self.lock:
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
    node = OpticalFlowFlightNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
