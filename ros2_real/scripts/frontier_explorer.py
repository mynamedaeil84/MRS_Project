#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
import random
import math


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Declare parameters
        self.declare_parameter('map_topic', '/map',
            ParameterDescriptor(description='OccupancyGrid topic for SLAM map'))

        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.get_logger().info(f"Subscribed to map topic: {self.map_topic}")

        # Initialize Nav2
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Subscribe to map
        self.subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.map_data = None
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.active_goal = False

    def map_callback(self, msg):
        self.map_data = msg

    def timer_callback(self):
        if self.active_goal:
            # Check if navigation is finished
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Reached frontier goal.")
                self.active_goal = False
            elif result == TaskResult.FAILED:
                self.get_logger().warn("Failed to reach frontier. Selecting another.")
                self.active_goal = False
            else:
                self.get_logger().info("Navigating...")
            return

        if not self.map_data:
            self.get_logger().info("Waiting for map...")
            return

        # Find frontiers
        frontiers = self.find_frontier()
        if not frontiers:
            self.get_logger().info('No frontiers found.')
            return

        robot_pose = self.navigator.getCurrentPose()
        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y

        # Choose closest frontier
        goal = min(frontiers, key=lambda f: math.hypot(f[0] - robot_x, f[1] - robot_y))
        self.get_logger().info(f"Navigating to frontier: {goal}")

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        pose.pose.orientation.w = 1.0  # No rotation

        self.navigator.goToPose(pose)
        self.active_goal = True

    def find_frontier(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        data = np.array(self.map_data.data).reshape((height, width))
        frontiers = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0 and -1 in data[y-1:y+2, x-1:x+2]:
                    wx = x * resolution + origin.x
                    wy = y * resolution + origin.y
                    frontiers.append((wx, wy))
        return frontiers


def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
