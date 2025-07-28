from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import rclpy

class ObstacleMarker(Node):
    def __init__(self):
        super().__init__('obstacle_marker')
        self.create_subscription(String, '/obstacle_info', self.cb, 10)
        self.pub = self.create_publisher(Marker, '/obstacle_marker', 10)

    def cb(self, msg):
        try:
            _, data = msg.data.split(":")
            x, y = map(float, data.split(",")[:2])

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.a = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1

            self.pub.publish(marker)
            self.get_logger().info(f"Marker Published at {x}, {y}")
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
