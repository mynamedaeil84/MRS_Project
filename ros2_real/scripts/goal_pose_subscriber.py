#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('goal_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.navigator = BasicNavigator()

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal: ({msg.pose.position.x}, {msg.pose.position.y})')
        self.navigator.goToPose(msg)
        result = self.navigator.waitUntilNavComplete()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully.')
        else:
            self.get_logger().warn('Failed to reach goal.')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
