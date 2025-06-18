#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
import numpy as np
import time

class FrontierExplorer(Node):
    def _init_(self):
        super()._init_('frontier_explorer')
        self.get_logger().info("🌍 Frontier Explorer node started")

        # Variables for retry logic and edge exploration
        self.failed_goal_attempts = 0
        self.max_failed_goal_attempts = 3
        self.goal_in_progress = False
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.latest_map = None
        self.goal_x = None  # Add this to store goal x-coordinate
        self.goal_y = None  # Add this to store goal y-coordinate

        self.wait_for_nav2_server()

    def wait_for_nav2_server(self):
        """ Wait for Nav2 action server """
        self.get_logger().info("⏳ Waiting for Nav2 action server (navigate_to_pose)...")

        while not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("🕒 Nav2 action server not ready. Retrying...")

    def map_callback(self, msg):
        """ Callback to process the latest map """
        self.get_logger().info("🗺 Map received")
        self.latest_map = msg

        if not self.goal_in_progress:
            self.find_and_navigate_to_frontier()

    def find_and_navigate_to_frontier(self):
        """ Find unexplored frontiers and navigate towards one """
        if self.latest_map is None:
            self.get_logger().warn("⚠ No map available for frontier detection")
            return

        width = self.latest_map.info.width
        height = self.latest_map.info.height
        data = np.array(self.latest_map.data).reshape((height, width))

        frontier_cells = np.argwhere(data == -1)
        if len(frontier_cells) == 0:
            self.get_logger().info("✅ No unexplored frontiers found")
            self.explore_edges()
            return

        # Select the first frontier cell found
        target_y, target_x = frontier_cells[0]
        self.get_logger().info(f"🎯 Frontier found at ({target_x}, {target_y})")

        origin = self.latest_map.info.origin.position
        res = self.latest_map.info.resolution
        goal_x = origin.x + target_x * res
        goal_y = origin.y + target_y * res

        self.send_navigation_goal(goal_x, goal_y)

    def send_navigation_goal(self, x, y):
        """ Send navigation goal to the robot """
        self.goal_x = x  # Store goal coordinates
        self.goal_y = y  # Store goal coordinates
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"📡 Sending goal to ({x:.2f}, {y:.2f})")
        self.goal_in_progress = True
        self.failed_goal_attempts = 0  # Reset failure counter on new goal

        send_goal_future = self.nav_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """ Callback for goal response """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            self.retry_navigation(self.goal_x, self.goal_y)
            return

        self.get_logger().info("✅ Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ Callback when goal is completed or failed """
        result = future.result().result
        self.get_logger().info(f"🧭 Goal finished with result: {result}")
        if result.error_code != 0:
            self.failed_goal_attempts += 1
            self.get_logger().warn(f"❌ Goal failed {self.failed_goal_attempts} times.")
            if self.failed_goal_attempts >= self.max_failed_goal_attempts:
                self.get_logger().warn(f"❌ Maximum retries reached for this goal. Aborting and finding a new frontier.")
                self.failed_goal_attempts = 0
                self.find_and_navigate_to_frontier()
        else:
            self.failed_goal_attempts = 0  # Reset after success
        self.goal_in_progress = False

    def retry_navigation(self, x, y):
        """ Retry navigation to a failed goal """
        self.get_logger().warn(f"❌ Goal failed. Retrying goal ({x}, {y})...")
        self.send_navigation_goal(x, y)

    def explore_edges(self):
        """ Explore the edges of the map if no frontiers are found """
        self.get_logger().info("🧭 No frontiers found. Exploring the edges of the map.")
        # You can implement a basic "edge exploration" by going around the edges of the map.
        width = self.latest_map.info.width
        height = self.latest_map.info.height
        resolution = self.latest_map.info.resolution
        origin = self.latest_map.info.origin.position

        # Define the corners of the map for edge exploration
        corners = [
            (origin.x, origin.y),  # Top-left
            (origin.x + width * resolution, origin.y),  # Top-right
            (origin.x, origin.y + height * resolution),  # Bottom-left
            (origin.x + width * resolution, origin.y + height * resolution),  # Bottom-right
        ]

        for corner in corners:
            self.get_logger().info(f"📡 Sending goal to edge at {corner}")
            self.send_navigation_goal(corner[0], corner[1])

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()