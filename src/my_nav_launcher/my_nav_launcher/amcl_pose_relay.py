#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class AmclPoseRelay(Node):
    def __init__(self):
        super().__init__('amcl_pose_relay')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,   # 通常AMCLはRELIABLE
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb, qos
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot1/amcl_pose', qos
        )
        self.count = 0
        self.get_logger().info('Relay started: /amcl_pose -> /robot1/amcl_pose')

    def cb(self, msg: PoseWithCovarianceStamped):
        self.count += 1
        self.pub.publish(msg)
        self.get_logger().info(f'Relayed AMCL pose #{self.count}  (stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec})')

def main():
    rclpy.init()
    node = AmclPoseRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
