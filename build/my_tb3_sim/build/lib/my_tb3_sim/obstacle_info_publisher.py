import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ObstacleInfoPublisher(Node):
    def __init__(self):
        super().__init__('obstacle_info_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pub = self.create_publisher(String, '/obstacle_info', qos_profile)
        self.trial_number = 1
        self.round = 1
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.round_timer = self.create_timer(300.0, self.advance_round)

    def timer_callback(self):
        if self.round <= 5:
            now = self.get_clock().now().to_msg()
            msg = String()
            msg.data = f"{now.sec}.{now.nanosec}:1.5,0.5:medium"
            self.pub.publish(msg)
            self.get_logger().info(f"[Round {self.round} - Trial {self.trial_number}] Published: {msg.data}")
            self.trial_number += 1

    def advance_round(self):
        self.round += 1
        if self.round > 5:
            self.get_logger().info("5 라운드(25분) 종료 → 자동 종료")
            self.destroy_node()
        else:
            self.get_logger().info(f"라운드 {self.round} 시작")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
