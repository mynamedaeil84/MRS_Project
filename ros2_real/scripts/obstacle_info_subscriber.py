import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from my_tb3_sim.qos_utils import load_qos_profile
import random

class ObstacleSubscriber(Node):
    def __init__(self):
        super().__init__('tb3_2_obstacle_handler')
        qos = load_qos_profile('obstacle_info')

        self.subscription = self.create_subscription(
            String,
            '/obstacle_info',
            self.obstacle_callback,
            qos
        )

        self.reaction_pub = self.create_publisher(Float32, '/reaction_delay', 10)
        self.last_obstacle_received_time = None

        self._action_client = ActionClient(self, NavigateToPose, '/tb3_2/navigate_to_pose')
        self.goal_active = False

    def obstacle_callback(self, msg):
        self.get_logger().info(f"📩 장애물 수신: {msg.data}")

        # 타임스탬프 파싱
        try:
            sent_time_str, _ = msg.data.split(":", 1)
            sec, nsec = map(float, sent_time_str.split("."))
            self.last_obstacle_received_time = sec + nsec * 1e-9
        except Exception as e:
            self.get_logger().warn(f"⚠️ 타임스탬프 파싱 실패: {e}")
            return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("⚠️ Nav2 서버 미응답. 장애물 회피 실패.")
            return

        # 현재 경로 취소
        if self.goal_active:
            self.get_logger().info("🛑 현재 경로 취소 요청")
            self._action_client.cancel_all_goals()

        # 새로운 목표 설정
        dx = random.uniform(-1.0, 1.0)
        dy = random.uniform(-1.0, 1.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = dx + 2.0
        goal_msg.pose.pose.position.y = dy + 2.0
        goal_msg.pose.pose.orientation.w = 1.0

        # 반응 지연 계산
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_obstacle_received_time:
            reaction_delay = now - self.last_obstacle_received_time
            self.get_logger().info(f"📊 반응 지연: {reaction_delay:.3f}초")
            self.reaction_pub.publish(Float32(data=reaction_delay))

        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"🚧 회피 목표 전송: x={goal_msg.pose.pose.position.x:.2f}, y={goal_msg.pose.pose.position.y:.2f}")
        self.goal_active = True

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
