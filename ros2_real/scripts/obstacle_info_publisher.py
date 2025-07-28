import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time
import time
from rclpy.qos import QoSProfile
from my_tb3_sim.qos_utils import load_qos_profile

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')     
        qos = load_qos_profile('obstacle_info')
        self.publisher = self.create_publisher(String, '/obstacle_info', qos)
        self.subscription = self.create_subscription(
             LaserScan,
             '/tb3_1/scan',
             self.laser_callback,
             10
        )

        self.threshold = 0.5  # 0.5m 이내 물체 감지 시 장애물로 판단

    def laser_callback(self, msg):
        if min(msg.ranges) < self.threshold:
            detect_time = self.get_clock().now().to_msg()
            info = f"Detected obstacle at distance {min(msg.ranges):.2f} m, time: {detect_time.sec}.{detect_time.nanosec}"
            self.get_logger().info(info)
            self.publisher.publish(String(data=info))

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()    
