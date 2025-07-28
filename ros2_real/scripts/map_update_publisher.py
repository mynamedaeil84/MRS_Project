#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapUpdatePublisher(Node):
    def __init__(self):
        super().__init__('map_update_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map_update', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.latest_map = None
        self.timer = self.create_timer(1.0, self.publish_map_update)

    def map_callback(self, msg):
        self.latest_map = msg

    def publish_map_update(self):
        if self.latest_map:
            self.publisher_.publish(self.latest_map)
            self.get_logger().info('Published map update')

def main(args=None):
    rclpy.init(args=args)
    node = MapUpdatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
