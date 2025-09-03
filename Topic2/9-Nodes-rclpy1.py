import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #Message/Topic/Buffer
        self.timer = self.create_timer(0.5, self.timer_callback) #0.5 seconds

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2    # forward speed (m/s)
        msg.angular.z = 0.1   # yaw rate (rad/s)
      
        self.publisher_.publish(msg)
        print(f'Publishing: v={msg.linear.x}, w={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
