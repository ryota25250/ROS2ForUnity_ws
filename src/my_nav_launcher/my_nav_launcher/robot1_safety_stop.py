# robot1_safety_stop.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SafetyStop(Node):
    def __init__(self):
        # namespace を /robot1 に固定（起動時に変えたい場合は __main__ のコメント参照）
        super().__init__('safety_stop', namespace='robot1')

        self.declare_parameter('front_center_deg', 180)  # 0=前方, 180=後方 など
        self.center_deg = float(self.get_parameter('front_center_deg').value)

        # --- パラメータ（必要に応じて ros2 param で上書き可） ---
        self.declare_parameter('forward_speed', 0.2)       # m/s
        self.declare_parameter('stop_distance', 0.50)       # m
        self.declare_parameter('resume_distance', 0.65)     # m（ヒステリシス用。stop より大きく）
        self.declare_parameter('front_fov_deg', 60.0)       # 前方何度を見るか（±half）
        self.declare_parameter('cmd_rate_hz', 10.0)

        self.v = float(self.get_parameter('forward_speed').value)
        self.stop_d = float(self.get_parameter('stop_distance').value)
        self.resume_d = float(self.get_parameter('resume_distance').value)
        self.fov_deg = float(self.get_parameter('front_fov_deg').value)
        self.cmd_dt = 1.0 / float(self.get_parameter('cmd_rate_hz').value)

        # QoS（センサ向け）
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher/Subscriber
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)  # -> /robot1/cmd_vel
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.on_scan, qos_sensor)

        # 制御ループ
        self.timer = self.create_timer(self.cmd_dt, self.tick)

        # 状態
        self.front_clear = True
        self.last_scan = None

        self.get_logger().info('Subscribing /robot1/scan, Publishing /robot1/cmd_vel')

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        # 前方FOV内の最小距離を求めて安全判定
        min_front = self._min_front_range(msg, self.fov_deg)

        if min_front is not None: self.get_logger().info(f"min_front={min_front:.3f} m")
        else: self.get_logger().info("min_front=None")

        if min_front is None:
            # 有効データなし → 念のため停止方向へ
            self.front_clear = False
            return

        # ヒステリシス：近づいたら停止、十分離れたら前進再開
        if self.front_clear and min_front <= self.stop_d:
            self.front_clear = False
        elif not self.front_clear and min_front >= self.resume_d:
            self.front_clear = True

    def tick(self):
        # 速度コマンド出力
        cmd = Twist()
        if self.front_clear:
            cmd.linear.x = self.v
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)

    def _min_front_range(self, scan: LaserScan, fov_deg: float):
        """前方±fov_deg/2 の扇形の有効レンジ最小値を返す。無ければ None。"""
        # 角度→index の計算
        half = math.radians(fov_deg / 2.0)
        center = math.radians(self.center_deg)
        # 前方(0rad)基準のインデックス範囲を作る
        # LaserScan は angle_min から angle_increment で並ぶ
        def angle_to_index(angle):
            return int(round((angle - scan.angle_min) / scan.angle_increment))

        i_min = max(0, min(len(scan.ranges) - 1, angle_to_index(center - half)))
        i_max = max(0, min(len(scan.ranges) - 1, angle_to_index(center + half)))
        if i_min > i_max:
            i_min, i_max = i_max, i_min

        vals = []
        for r in scan.ranges[i_min:i_max + 1]:
            if r is None:
                continue
            if math.isfinite(r) and r > scan.range_min:  # 0や極端なノイズを除外
                vals.append(r)

        return min(vals) if vals else None


def main():
    rclpy.init()
    node = SafetyStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
