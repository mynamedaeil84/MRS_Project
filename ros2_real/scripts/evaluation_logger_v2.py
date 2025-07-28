import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
import csv
import os
from datetime import datetime
from std_msgs.msg import String, Bool, Float32  # Float32 추가

class EvaluationLoggerV2(Node):
    def __init__(self):
        super().__init__('evaluation_logger_v2')

        self.start_time = self.get_clock().now()
        self.log_path = os.path.expanduser('~/ros2_ws/evaluation_log_v2.csv')
        self.create_csv()
        self.reaction_delays = []  # 🔹 추가
        self.obstacle_count = 0
        self.hazard_count = 0
        self.task_count = 0
        self.goal_reached = False

        self.coverage = 0.0
        self.total_map_cells = 0
        self.explored_cells = 0
        self.map_data = None
        self.delay_list = []

        self.create_subscription(String, '/obstacle_info', self.obstacle_cb, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/hazard_zone', self.hazard_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/task_allocation', self.task_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/goal_reached', self.goal_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/tb3_1/map', self.map_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, '/reaction_delay', self.reaction_cb, 10)  # 🔹 구독 추가
        self.timer = self.create_timer(10.0, self.summarize)

    def reaction_cb(self, msg):
        self.reaction_delays.append(msg.data)
        self.log_event('ReactionDelay', f"{msg.data:.3f}")

    def create_csv(self):
        with open(self.log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'event', 'detail'])

    def log_event(self, event, detail):
        timestamp = datetime.now().isoformat()
        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, event, detail])
        self.get_logger().info(f"[LOG] {event}: {detail}")

    def obstacle_cb(self, msg):
        self.obstacle_count += 1
        try:
            sent_time_str, _ = msg.data.split(':', 1)
            sent_sec, sent_nsec = map(float, sent_time_str.split('.'))
            sent_time = sent_sec + sent_nsec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            delay = now - sent_time
            self.delay_list.append(delay)
            self.log_event('ObstacleDetected', f"delay={delay:.3f}s")
        except Exception:
            self.log_event('ObstacleDetected', "no timestamp")

    def hazard_cb(self, msg):
        self.hazard_count += 1
        pos = msg.point
        self.log_event('HazardZone', f"({pos.x:.2f}, {pos.y:.2f})")

    def task_cb(self, msg):
        self.task_count += 1
        self.log_event('TaskAssigned', msg.data)

    def goal_cb(self, msg):
        self.goal_reached = msg.data
        self.log_event('GoalReached', f"success={msg.data}")

    def map_cb(self, msg):
        self.map_data = msg.data
        explored = sum(1 for cell in self.map_data if cell != -1)
        total = len(self.map_data)
        if total > 0:
            self.coverage = (explored / total) * 100.0

    def summarize(self):
        now = self.get_clock().now()
        duration = (now - self.start_time).nanoseconds * 1e-9
        avg_delay = sum(self.delay_list) / len(self.delay_list) if self.delay_list else 0.0

        self.get_logger().info("🧾 성능 요약")
        self.get_logger().info(f"⏱ 총 시간: {duration:.1f}s")
        self.get_logger().info(f"📦 작업 수: {self.task_count}")
        self.get_logger().info(f"🚧 장애물 수: {self.obstacle_count}")
        self.get_logger().info(f"⚠️ 위험지역 수: {self.hazard_count}")
        self.get_logger().info(f"🧭 임무 성공: {self.goal_reached}")
        self.get_logger().info(f"📊 커버리지: {self.coverage:.2f}%")
        self.get_logger().info(f"📡 평균 지연: {avg_delay:.3f}s")

def main(args=None):
    rclpy.init(args=args)
    node = EvaluationLoggerV2()
    rclpy.spin(node)
    rclpy.shutdown()

