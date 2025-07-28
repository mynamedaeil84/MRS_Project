import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy
import csv

class ReactionLogger(Node):
    def __init__(self):
        super().__init__('reaction_logger')
        self.qos_profile = 'RELIABLE'
        self.trial_number = 1
        self.round = 1
        self.csv = open('reaction_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['Round', 'Trial', 'QoS Profile', 'Detected Time', 'Received Time', 'Delay (s)', 'React Time (s)', 'Missed Msg', 'Note'])

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.sub = self.create_subscription(String, '/obstacle_info', self.callback, qos)
        self.reaction_delay_pub = self.create_publisher(Float32, '/reaction_delay', 10)
        self.round_timer = self.create_timer(300.0, self.advance_round)

    def callback(self, msg):
        if self.round <= 5:
            now = self.get_clock().now().nanoseconds * 1e-9
            try:
                sent_time_str, _ = msg.data.split(":", 1)
                sec, nsec = map(float, sent_time_str.split("."))
                sent_time = sec + nsec * 1e-9
                delay = now - sent_time
                react_time = 0.2
                self.writer.writerow([self.round, self.trial_number, self.qos_profile, f"{sent_time:.3f}", f"{now:.3f}", f"{delay:.3f}", f"{react_time:.3f}", 0, 'OK'])
                self.reaction_delay_pub.publish(Float32(data=delay))
                self.get_logger().info(f"[Round {self.round} - Trial {self.trial_number}] Delay: {delay:.3f}s")
            except Exception as e:
                self.writer.writerow([self.round, self.trial_number, self.qos_profile, "—", "—", "—", "—", 1, 'Loss'])
                self.get_logger().warn(f"Message parse error: {e}")
            self.trial_number += 1

    def advance_round(self):
        self.round += 1
        if self.round > 5:
            self.get_logger().info("5 라운드(25분) 수신 종료 → 자동 종료")
            self.destroy_node()
        else:
            self.get_logger().info(f"라운드 {self.round} 시작")
            self.trial_number = 1

    def destroy_node(self):
        self.csv.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ReactionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
