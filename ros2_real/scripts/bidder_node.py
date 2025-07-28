import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import math

class Bidder(Node):
    def __init__(self):
        super().__init__('bidder_tb3_2')
        self.robot_id = 'tb3_2'
        self.create_subscription(String, '/auction_task', self.task_callback, 10)
        self.publisher = self.create_publisher(String, '/auction_bid', 10)

    def task_callback(self, msg):
        parts = dict(p.split(":") for p in msg.data.split(", "))
        task_id = int(parts["task_id"])
        x, y = float(parts["x"]), float(parts["y"])

        # 가상의 로봇 위치로부터 거리 계산 (ex: TB3-2 at (0,0))
        cost = math.sqrt(x**2 + y**2)
        bid_msg = String()
        bid_msg.data = f"task_id:{task_id}, robot_id:{self.robot_id}, cost:{cost:.2f}"
        self.publisher.publish(bid_msg)
        self.get_logger().info(f"[📤] 입찰 전송: {bid_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Bidder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
