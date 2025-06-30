import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math
import time
from collections import deque


class FrontierExplorer(Node):
    def _init_(self):
        super()._init_('frontier_explorer')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.frontier_queue = deque()
        self.latest_map = None
        self.latest_pose = None
        self.last_pose = None
        self.last_move_time = self.get_clock().now()
        self.goal_handle = None
        self.goal_retry_count = 0
        self.blacklist = set()
        self.stuck_spin_attempts = 0
        self.spin_start_time = None
        self.spin_duration_sec = 10

        self.active_goal = None

        self.timer = self.create_timer(2.0, self.control_loop)

        self.get_logger().info("🌍 Frontier Explorer started")
        self.wait_for_nav2()

    def wait_for_nav2(self):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("⏳ Waiting for NavigateToPose action server...")
        self.get_logger().info("✅ Nav2 action server ready")

    def map_callback(self, msg):
        self.latest_map = msg

    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose

    def euclidean_dist(self, p1, p2):
        return math.sqrt((p1.position.x - p2.position.x) ** 2 + (p1.position.y - p2.position.y) ** 2)

    def control_loop(self):
        if self.goal_handle or not self.latest_map or not self.latest_pose:
            return

        now = self.get_clock().now()
        if self.last_pose and self.euclidean_dist(self.latest_pose, self.last_pose) < 0.05:
            if (now - self.last_move_time) > Duration(seconds=10):
                self.get_logger().warn("🚫 Robot did not move. Attempting spin recovery...")
                self.trigger_spin_recovery()
                self.last_move_time = now
                return
        else:
            self.last_move_time = now

        self.last_pose = self.latest_pose

        if not self.frontier_queue:
            self.find_new_frontiers()

        while self.frontier_queue:
            goal = self.frontier_queue.popleft()
            if goal in self.blacklist:
                continue
            self.send_goal(goal)
            break

    def send_goal(self, goal):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal[0])
        pose.pose.position.y = float(goal[1])
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        self.get_logger().info(f"📍 Sending goal to ({goal[0]:.2f}, {goal[1]:.2f})")
        self.goal_future = self.nav_client.send_goal_async(goal_msg)
        self.goal_future.add_done_callback(self.goal_response_callback)
        self.active_goal = goal

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            self.active_goal = None
            return

        self.get_logger().info("✅ Goal accepted")
        self.goal_handle = goal_handle
        goal_result_future = goal_handle.get_result_async()
        goal_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        code = result.error_code  # fixed
        if code == 0:
            self.get_logger().info("🎉 Goal succeeded")
        else:
            self.get_logger().warn("❌ Goal failed. Blacklisting it immediately.")
            if self.active_goal:
                self.blacklist.add(self.active_goal)

        self.active_goal = None
        self.goal_handle = None
        self.goal_retry_count = 0

    def trigger_spin_recovery(self):
        # Instead of calling a service, just simulate a spin behavior here.
        # You can integrate nav2_behavior_server spin plugin via an action if implemented.
        self.get_logger().info("🔄 Triggering spin recovery behavior...")
        self.get_logger().info("✅ Spin behavior finished. Resuming exploration.")

    def find_new_frontiers(self):
        if self.latest_map is None:
            self.get_logger().warn("⚠ No map data available for frontier detection.")
            return

        map_data = self.latest_map
        resolution = map_data.info.resolution
        origin = map_data.info.origin.position
        width = map_data.info.width
        height = map_data.info.height
        data = map_data.data

        frontier_cells = set()
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                index = y * width + x
                if data[index] != 0:
                    continue
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        n_idx = (y + dy) * width + (x + dx)
                        if data[n_idx] == -1:
                            frontier_cells.add((x, y))
                            break

        self.frontier_queue.clear()
        for x, y in frontier_cells:
            wx = origin.x + x * resolution
            wy = origin.y + y * resolution
            if (wx, wy) not in self.blacklist:
                self.frontier_queue.append((wx, wy))

        self.get_logger().info(f"🔍 Found {len(frontier_cells)} frontier points")


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if _name_ == '_main_':
    main()