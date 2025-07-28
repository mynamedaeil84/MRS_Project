import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import os, time, random

class ObstacleInfoPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_info_publisher')

        # 환경 변수에서 QoS 설정 읽기
        reliability = os.environ.get("QOS_RELIABILITY", "RELIABLE")
        durability = os.environ.get("QOS_DURABILITY", "VOLATILE")
        depth = int(os.environ.get("QOS_DEPTH", "10"))

        qos_profile = QoSProfile(
            reliability=getattr(ReliabilityPolicy, reliability),
            durability=getattr(DurabilityPolicy, durability),
            history=HistoryPolicy.KEEP_LAST,
            depth=depth
        )

        self.pub = self.create_publisher(String, '/obstacle_info', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.round = 1
        self.round_timer = self.create_timer(300.0, self.advance_round)

    def timer_callback(self):
        if self.round <= 5:
            now_ns = self.get_clock().now().nanoseconds
            time.sleep(random.uniform(0.005, 0.025))  # 5~25ms 랜덤 지연
            msg = String()
            msg.data = f"{now_ns}:1.5,0.5:medium"
            self.pub.publish(msg)
            self.get_logger().info(f"[Round {self.round}] Published: {msg.data}")

    def advance_round(self):
        self.round += 1
        if self.round > 5:
            self.get_logger().info("5 라운드 종료 → 퍼블리셔 종료")
            self.destroy_node()
        else:
            self.get_logger().info(f"라운드 {self.round} 시작")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
