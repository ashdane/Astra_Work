import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool,SetMode
from rclpy.duration import Duration
from math import sin,cos,pi
class OffboardControlNode(Node):
     def __init__(self):
          self.LandSent = False
          super().__init__('offb_node_py')

          self.current_state=State()

          self.state_sub=self.create_subscription(State,'mavros/state',self.state_cb,10)

          #self.local_pos_pub=self.create_publisher(PoseStamped,'mavros/setpoint_position/local',10)
          self.vel_pub = self.create_publisher(Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10) #gonna publish to vel topic instead of pos topic

          self.arming_client=self.create_client(CommandBool,'mavros/cmd/arming')

          self.set_mode_client=self.create_client(SetMode,'mavros/set_mode')

          while not self.arming_client.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('Waiting for arming service')
          
          while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('Waiting for set_mode service')

          # self.pose=PoseStamped()
          # self.pose.pose.position.x=0.0
          # self.pose.pose.position.y=0.0
          # self.pose.pose.position.z=0.0
          self.vel_cmd = Twist()
          self.vel_cmd.linear.x = 0.0  # Velocity in the x-direction (m/s)
          self.vel_cmd.linear.y = 0.0
          self.vel_cmd.linear.z = 1.0  # Ascent velocity (m/s)

          self.last_req=self.get_clock().now()

          self.timer=self.create_timer(0.05,self.timer_callback)

          #self.radius=10.0
          #self.Theta=0
          self.dt=0.20

     def state_cb(self,msg):
          self.current_state=msg
          self.get_logger().info(f"Current flight mode: {self.current_state.mode}")

     def response_callback(self,future):
           try:
               response=future.result()
               self.get_logger().info(f'Result of MAV_CMD:{response}')
           except Exception as e:
                self.get_logger().error(f'Service call failed:{str(e)}')

     def land_drone(self):
          if self.current_state.mode != 'AUTO.LAND':
               land_set_mode = SetMode.Request()
               land_set_mode.custom_mode = 'AUTO.LAND'
               future = self.set_mode_client.call_async(land_set_mode)
               future.add_done_callback(self.response_callback)
               self.get_logger().info("Landing command sent, awaiting mode switch.")
               self.LandSent = True
               self.last_req=self.get_clock().now()

     def timer_callback(self):
          #self.get_logger().info(f"Current position: x={self.pose.pose.position.x}, z={self.pose.pose.position.z}")
          if self.current_state.mode !='OFFBOARD' and (self.get_clock().now()-self.last_req)> Duration(seconds=5) and self.LandSent==False:
               offb_set_mode=SetMode.Request()
               offb_set_mode.custom_mode='OFFBOARD'
               future=self.set_mode_client.call_async(offb_set_mode)
               future.add_done_callback(self.response_callback)

               self.last_req=self.get_clock().now()
          elif self.current_state.mode =='OFFBOARD' and (self.get_clock().now() - self.last_req)>Duration(seconds=30):
               self.get_logger().info("Landing condition met, sending land command.")
               self.land_drone()
          else:
               if not self.current_state.armed and (self.get_clock().now() - self.last_req)>Duration(seconds=5):
                    arm_cmd=CommandBool.Request()
                    arm_cmd.value=True
                    future=self.arming_client.call_async(arm_cmd)
                    future.add_done_callback(self.response_callback)

                    self.last_req=self.get_clock().now()
                    
          if self.current_state.mode == 'OFFBOARD':
               if (self.get_clock().now()-self.last_req)> Duration(seconds=10):
                    self.vel_cmd.linear.z = 0.0
                    self.vel_cmd.linear.x = 0.0
               if (self.get_clock().now()-self.last_req)> Duration(seconds=20):
                    self.vel_cmd.linear.x += self.dt
          #self.pose.pose.position.x=self.radius*cos(self.Theta)
          #self.pose.pose.position.y=self.radius*sin(self.Theta)
          #self.pose.pose.position.z=2.0
          if not self.LandSent:
               self.vel_pub.publish(self.vel_cmd)

          #self.Theta=self.Theta+self.dt

def main(args=None):
     rclpy.init(args=args)
     node= OffboardControlNode()           

     rclpy.spin(node)

     node.destroy_node()

     rclpy.shutdown()

if __name__=='__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")           
                    
          
     
