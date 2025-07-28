import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class Auctioneer(Node):
    def __init__(self):
        super().__init__('auctioneer')
        self.pub_task = self.create_publisher(String, '/auction_task', 10)
        self.sub_bid = self.create_subscription(String, '/auction_bid', self.bid_callback, 10)
        self.pub_assign = self.create_publisher(String, '/task_assignment', 10)

        self.bids = {}
        self.current_task_id = 0
        self.timer = self.create_timer(15.0, self.publish_task)

    def publish_task(self):
        self.bids.clear()
        self.current_task_id += 1
        x, y = random.uniform(-5, 5), random.uniform(-5, 5)
        msg = String()
        msg.data = f"task_id:{self.current_task_id}, x:{x:.2f}, y:{y:.2f}"
        self.pub_task.publish(msg)
        self.get_logger().info(f"[📢] 경매 시작: {msg.data}")

    def bid_callback(self, msg):
        self.get_logger().info(f"[📨] 입찰 수신: {msg.data}")
        parts = dict(p.split(":") for p in msg.data.split(", "))
        task_id = int(parts["task_id"])
        if task_id != self.current_task_id:
            return
        robot = parts["robot_id"]
        cost = float(parts["cost"])
        self.bids[robot] = cost

        # 모든 입찰자가 응답했다고 가정하고 낙찰자 선정
        if len(self.bids) >= 1:  # 확장 시 조건 조정
            winner = min(self.bids, key=self.bids.get)
            assign_msg = String()
            assign_msg.data = f"task_id:{task_id}, winner:{winner}"
            self.pub_assign.publish(assign_msg)
            self.get_logger().info(f"[🏁] 낙찰자: {assign_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Auctioneer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
