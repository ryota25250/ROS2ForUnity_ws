# robot1_cmd_vel_pub.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        # ここで namespace を robot1 に設定
        super().__init__('cmd_vel_publisher', namespace='robot1')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)  # -> /robot1/cmd_vel
        self.timer = self.create_timer(0.1, self.tick)  # 10Hz
        self.get_logger().info('Publishing to /robot1/cmd_vel')

    def tick(self):
        msg = Twist()
        msg.linear.x = 0.2   # 前進 0.2 m/s
        msg.angular.z = 0.0  # 旋回なし
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
