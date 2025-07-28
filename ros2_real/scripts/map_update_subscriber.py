#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapUpdateSubscriber(Node):
    def __init__(self):
        super().__init__('map_update_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map_update', self.map_update_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

    def map_update_callback(self, msg):
        self.map_pub.publish(msg)
        self.get_logger().info('Received and republished map update')

def main(args=None):
    rclpy.init(args=args)
    node = MapUpdateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
