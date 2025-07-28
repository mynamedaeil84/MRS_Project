import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from my_tb3_sim.qos_utils import load_qos_profile

class HazardZonePublisher(Node):
    def __init__(self):
        super().__init__('hazard_zone_publisher')
        qos = load_qos_profile('hazard_zone')
        self.publisher = self.create_publisher(PointStamped, '/hazard_zone', qos)
        self.timer = self.create_timer(10.0, self.publish_hazard_zone)

    def publish_hazard_zone(self):
        # 미끄러운 지역 예시 좌표: (-3.0, 3.0)
        hazard_point = PointStamped()
        hazard_point.header.frame_id = 'map'
        hazard_point.header.stamp = self.get_clock().now().to_msg()
        hazard_point.point.x = -3.0
        hazard_point.point.y = 3.0
        hazard_point.point.z = 0.0

        self.publisher.publish(hazard_point)
        self.get_logger().info("Published hazard zone at (-3.0, 3.0)")

def main(args=None):
    rclpy.init(args=args)
    node = HazardZonePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
