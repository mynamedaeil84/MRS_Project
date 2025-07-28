import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker

class ObstacleMarker(Node):
    def __init__(self):
        super().__init__('obstacle_marker')
        self.create_subscription(String, '/obstacle_info', self.cb, 10)
        self.pub = self.create_publisher(Marker, '/obstacle_marker', 10)
        self.marker_id = 0

    def cb(self, msg):  # ✅ 반드시 클래스 내부에 정의돼야 함
        try:
            parts = msg.data.split(":")
            if len(parts) < 2:
                raise ValueError("Message format too short")

            coords = parts[1]
            x, y = map(float, coords.split(",")[:2])

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 30
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0

            self.pub.publish(marker)
            self.get_logger().info(f"Published marker[{self.marker_id}] at ({x}, {y})")
            self.marker_id += 1
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()