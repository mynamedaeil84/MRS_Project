import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy
import csv
from datetime import datetime

class ResultLogger(Node):
    def __init__(self):
        super().__init__('result_logger')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.subscription = self.create_subscription(
            String, '/obstacle_info', self.listener_callback, qos)

        self.logfile = open('reaction_log.csv', 'a', newline='')
        self.writer = csv.writer(self.logfile)
        self.writer.writerow(['Timestamp', 'Sent Time', 'Received Time', 'Delay'])

    def listener_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            sent_time_str, _ = msg.data.split(":", 1)
            sec, nsec = map(float, sent_time_str.split("."))
            sent_time = sec + nsec * 1e-9
            delay = now - sent_time
            self.writer.writerow([datetime.now().isoformat(), sent_time, now, delay])
            self.get_logger().info(f"Logged delay: {delay:.3f}s")
        except Exception as e:
            self.get_logger().error(f"Error parsing message: {e}")

    def destroy_node(self):
        self.logfile.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ResultLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
