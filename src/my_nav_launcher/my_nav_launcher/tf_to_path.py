#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

class TfToPath(Node):
    def __init__(self):
        super().__init__('tf_to_path')

        # どの座標系から base_link を見るか（map か odom）
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('topic', 'base_link_path')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('max_poses', 5000)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame  = self.get_parameter('child_frame').get_parameter_value().string_value
        self.topic        = self.get_parameter('topic').get_parameter_value().string_value
        self.rate_hz       = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.max_poses     = self.get_parameter('max_poses').get_parameter_value().integer_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Path, self.topic, 10)

        self.path = Path()
        self.path.header.frame_id = self.parent_frame

        period = 1.0 / max(self.rate_hz, 1e-6)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'Publishing Path on "{self.topic}" from TF {self.parent_frame} -> {self.child_frame}'
        )

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.parent_frame
        ps.pose.position.x = t.transform.translation.x
        ps.pose.position.y = t.transform.translation.y
        ps.pose.position.z = t.transform.translation.z
        ps.pose.orientation = t.transform.rotation

        self.path.header.stamp = ps.header.stamp
        self.path.poses.append(ps)

        # 長くなりすぎないように間引き
        if len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses:]

        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = TfToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
