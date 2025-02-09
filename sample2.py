import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')

        self.state = State()
        self.current_global_pose = NavSatFix()
        self.initial_reached = False  # To track if the initial GPS target is reached

        # Create callback groups
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.cb_2 = MutuallyExclusiveCallbackGroup()
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        # Create subscribers and publishers
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10,
            callback_group=self.callback_group
        )

        self.global_pose_sub = self.create_subscription(
            NavSatFix,
            'mavros/global_position/global',
            self.global_pose_cb,
            qos_profile,
            callback_group=self.callback_group
        )

        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )
        self.global_pos_pub = self.create_publisher(
            GeoPoseStamped,
            'mavros/setpoint_position/global',
            10
        )

        # Create service clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        # Wait for services to be available
        self.wait_for_service(self.arming_client)
        self.wait_for_service(self.set_mode_client)

        # Prepare service requests
        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True

        self.last_req = self.get_clock().now()

        # Set target GPS coordinates
        self.target_gps_pose = GeoPoseStamped()
        self.target_gps_pose.pose.position.latitude = 13.2867149
        self.target_gps_pose.pose.position.longitude = 77.5963487
        self.target_gps_pose.pose.position.altitude = 502.0

        # Set local position 5 meters ahead (in x direction)
        self.target_local_pose = PoseStamped()
        self.target_local_pose.pose.position.x = 0.0
        self.target_local_pose.pose.position.y = 5.0
        self.target_local_pose.pose.position.z = 15.0

        # Main loop


        self.timer = self.create_timer(0.1,self.main_loop,callback_group=self.cb_2)
        
        
    def state_cb(self, msg):
        self.state = msg

    def global_pose_cb(self, msg):
        self.current_global_pose = msg
        

    def wait_for_service(self, client):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def gps_reached(self):
        # Check if current GPS position is close to target GPS position (within a tolerance)
        lat_diff = abs(self.current_global_pose.latitude - self.target_gps_pose.pose.position.latitude)
        lon_diff = abs(self.current_global_pose.longitude - self.target_gps_pose.pose.position.longitude)
        alt_diff = abs(self.current_global_pose.altitude - self.target_gps_pose.pose.position.altitude)
        # self.get_logger().info(f"current lat: {self.current_global_pose.pose.position.latitude}")
        # self.get_logger().info(f"Latitude difference: {lat_diff}")
        # self.get_logger().info(f"Longitude difference: {lon_diff}")

        return lat_diff < 0.0001 and lon_diff < 0.0001
    
    

    def set_offb(self):
        self.hold = False
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = 'OFFBOARD'
        
        self.get_logger().info("Setting OFFBOARD mode...")
        future = self.set_mode_client.call_async(set_mode_request)
        future.add_done_callback(self.set_mode_callback)

    def set_mode_callback(self, future):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info("OFFBOARD mode set successfully.")
                self.perform_takeoff()
            else:
                self.get_logger().error("Failed to set OFFBOARD mode.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def arm_drone(self):



        
        client = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        
        msg = CommandBool.Request()
        msg.value = True

        future = client.call_async(msg)
        future.add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        try:
            if future.result().success:
                self.get_logger().info("Drone Armed and Ready.")
            else:
                self.get_logger().error("Arming failed!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    

    def main_loop(self):

        now = self.get_clock().now()

        # Switch to OFFBOARD mode if not already enabled
        if self.state.mode != "OFFBOARD" and (now - self.last_req).nanoseconds / 1e9 > 5.0:
            self.set_offb()

        # Arm the vehicle if not already armed
        elif not self.state.armed and (now - self.last_req).nanoseconds / 1e9 > 5.0:
            self.arm_drone()
            
            
        # Check if the initial GPS position is reached
        if not self.initial_reached:
            self.global_pos_pub.publish(self.target_gps_pose)
            if self.gps_reached():
                self.initial_reached = True
                self.get_logger().info("Initial GPS target reached. Switching to local frame.")
        else:
            # Publish 5 meters ahead in local frame
            self.local_pos_pub.publish(self.target_local_pose)

        self.get_clock().sleep_for(Duration(seconds=0.05))

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the node
    node = OffboardNode()

    # Create a multithreaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        # Spin with the executor
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
