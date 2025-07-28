import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import csv
import os

class ReactionLogger(Node):
    def __init__(self):
        super().__init__('reaction_logger')

        # 환경 변수에서 QoS 이름 가져오기
        self.qos_profile = os.environ.get("QOS_PROFILE_NAME", "Unknown")
        self.round = 1
        self.trial_number = 1
        self.last_obstacle_ns = None

        self.csv = open('reaction_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['Round', 'Trial', 'QoS Profile', 'Detected Time', 'Received Time', 'Delay (s)', 'React Time (s)', 'Missed Msg', 'Note'])

        self.sub = self.create_subscription(String, '/obstacle_info', self.callback, 10)
        self.reaction_time_sub = self.create_subscription(Float32, '/reaction_time', self.reaction_cb, 10)
        self.round_timer = self.create_timer(300.0, self.advance_round)

    def callback(self, msg):
        if self.round <= 5:
            try:
                sent_ns, _ = msg.data.split(":", 1)
                sent_ns = int(sent_ns)
                now_ns = self.get_clock().now().nanoseconds
                delay = (now_ns - sent_ns) * 1e-9
                self.last_obstacle_ns = sent_ns
                self.writer.writerow([self.round, self.trial_number, self.qos_profile, sent_ns * 1e-9, now_ns * 1e-9, delay, "", 0, "OK"])
                self.trial_number += 1
            except Exception as e:
                self.writer.writerow([self.round, self.trial_number, self.qos_profile, "—", "—", "—", "—", 1, f"Loss: {e}"])
                self.trial_number += 1

    def reaction_cb(self, msg):
        reaction_time = msg.data
        if self.trial_number > 1:
            self.writer.writerow([self.round, self.trial_number - 1, self.qos_profile, "", "", "", reaction_time, "", "reaction recorded"])

    def advance_round(self):
        self.round += 1
        if self.round > 5:
            self.get_logger().info("5 라운드 종료 → 로거 종료")
            self.csv.close()
            self.destroy_node()
        else:
            self.trial_number = 1
            self.get_logger().info(f"라운드 {self.round} 시작")

def main(args=None):
    rclpy.init(args=args)
    node = ReactionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
