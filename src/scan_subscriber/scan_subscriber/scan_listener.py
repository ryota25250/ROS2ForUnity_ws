import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanSubscriberNode(Node):
    def __init__(self):
        super().__init__('scan_subscriber_node')
        self.get_logger().info('Listening for a single /scan message...')

        # /scanトピックを購読するサブスクライバを作成
        # メッセージを受信したら listener_callback を呼び出す
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        """
        メッセージを1回受信したら呼び出される関数
        """
        self.get_logger().info('--- Received one /scan message ---')

        # 受信したメッセージの主要な内容を表示
        print(f"Header:")
        print(f"  Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns")
        print(f"  Frame ID: {msg.header.frame_id}")
        print(f"Angle Min: {msg.angle_min}")
        print(f"Angle Max: {msg.angle_max}")
        print(f"Angle Increment: {msg.angle_increment}")
        print(f"Range Min: {msg.range_min}")
        print(f"Range Max: {msg.range_max}")

        # ranges配列は長いため、最初の50個だけ表示
        if len(msg.ranges) > 50:
            print(f"Ranges (first 50): {msg.ranges[:50]}")
        else:
            # 配列が50個未満の場合はすべて表示
            print(f"Ranges: {msg.ranges}")

        self.get_logger().info('--- Shutting down node ---')

        # 1回受信したらノードを終了する
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    scan_subscriber = ScanSubscriberNode()

    # ノードを実行し、コールバックが呼ばれるまで待機
    rclpy.spin(scan_subscriber)

if __name__ == '__main__':
    main()