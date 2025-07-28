#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(10.0, self.publish_goal_pose)  # 예: 10초마다 전송

    def publish_goal_pose(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 5.0   # 예: 목표 위치 X
        goal.pose.position.y = 5.0   # 예: 목표 위치 Y
        goal.pose.orientation.w = 1.0
        self.publisher_.publish(goal)
        self.get_logger().info('Published goal pose')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
