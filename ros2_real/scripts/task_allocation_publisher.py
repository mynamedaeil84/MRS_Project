import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import random

class TaskAllocationPublisher(Node):
    def __init__(self):
        super().__init__('task_allocator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=5
        )

        self.publisher = self.create_publisher(String, '/task_allocation', qos)
        self.timer = self.create_timer(15.0, self.publish_task)

        self.task_id = 0
        self.candidate_positions = [
            (2.0, 3.0),
            (-4.0, -3.5),
            (5.0, -2.0),
            (-1.5, 4.5)
        ]

    def publish_task(self):
        self.task_id += 1
        x, y = random.choice(self.candidate_positions)
        priority = random.randint(1, 3)  # 예시: 1 = 최고 우선순위

        msg = String()
        msg.data = f"task_id:{self.task_id}, x:{x}, y:{y}, priority:{priority}"
        self.publisher.publish(msg)

        self.get_logger().info(f"Published task: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocationPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()

