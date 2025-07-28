import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class GoalMonitor(Node):
    def __init__(self):
        super().__init__('goal_monitor')
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/tb3_2/goal_pose',
            self.goal_cb,
            10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tb3_2/amcl_pose',
            self.pose_cb,
            10)
        self.pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.target = None
        self.tolerance = 0.3  # 30cm 내 접근 시 도달로 간주

    def goal_cb(self, msg):
        self.target = msg.pose.position
        self.get_logger().info(f"🎯 목표 위치 설정: ({self.target.x:.2f}, {self.target.y:.2f})")

    def pose_cb(self, msg):
        if self.target is None:
            return
        current = msg.pose.position
        dx = self.target.x - current.x
        dy = self.target.y - current.y
        dist = (dx**2 + dy**2)**0.5
        if dist < self.tolerance:
            self.pub.publish(Bool(data=True))
            self.get_logger().info("✅ 목표에 도달했습니다.")
            self.target = None

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitor()
    rclpy.spin(node)
    rclpy.shutdown()
