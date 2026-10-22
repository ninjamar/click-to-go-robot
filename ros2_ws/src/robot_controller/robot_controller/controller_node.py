import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ControllerNode(Node):
    def __init__(self):
        # Register this node with the ROS2 graph under the name "robot_controller".
        super().__init__("robot_controller")

        self.declare_parameter("emergency_stop_distance", 0.3)
        # Lidar angle (deg) that points at the robot's front, and the half-width of the
        # cone checked in the direction of travel. Wheels sit at the sides, outside it.
        self.declare_parameter("scan_front_angle_deg", 180.0)
        self.declare_parameter("emergency_cone_deg", 40.0)
        self._p_stop_dist = self.get_parameter("emergency_stop_distance").value
        self._p_front_angle = self.get_parameter("scan_front_angle_deg").value
        self._p_cone = self.get_parameter("emergency_cone_deg").value

        self._navigating = False
        self._linear_x = 0.0  # latest commanded forward speed
        self._goal_handle = None  # holds the active Nav2 goal so we can cancel it

        # An ActionClient sends goals to an action server and receives structured feedback.
        # Actions differ from topics: the caller gets confirmation that the goal was accepted,
        # periodic progress updates while it runs, and a final success/failure result.
        # Nav2's "navigate_to_pose" action runs the full planning + control pipeline
        # (global path, local controller, obstacle avoidance) in response to a single goal.
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # /goal_pose is published by RViz2's "2D Nav Goal" button. Clicking on the map
        # sends one PoseStamped — a position + orientation tagged with a coordinate frame.
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # /scan_filtered is the lidar output — an array of distance readings at evenly-spaced
        # angles around the robot. Used here only as a last-resort safety check;
        # Nav2's local costmap handles normal obstacle avoidance.
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan, 10)

    def _on_goal(self, msg: PoseStamped) -> None:
        """Subscription callback: called whenever a new navigation goal arrives."""
        if self._navigating and self._goal_handle:
            self.get_logger().info("replacing current goal")
            # cancel_goal_async() asks the action server to cancel without blocking.
            # _on_result will fire with a cancelled status when Nav2 acknowledges it.
            self._goal_handle.cancel_goal_async()

        if not self._nav_client.server_is_ready():
            self.get_logger().error("Nav2 not available")
            return

        x, y = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f"navigating to ({x:.2f}, {y:.2f})")
        self._navigating = True

        goal = NavigateToPose.Goal()
        goal.pose = msg

        # send_goal_async() returns a Future immediately without blocking the executor.
        # add_done_callback registers a function to call when Nav2 accepts or rejects
        # the goal — this is how ROS2 actions avoid blocking the single-threaded executor.
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        """Action callback: called when Nav2 decides whether to accept the goal."""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("goal rejected by Nav2")
            self._navigating = False
            return
        self._goal_handle = handle
        # get_result_async() returns another Future that fires when navigation finishes.
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        """Action callback: called periodically by Nav2 while navigation is in progress."""
        dist = feedback_msg.feedback.distance_remaining
        # throttle_duration_sec prevents log spam — at most one log line every 2 seconds.
        self.get_logger().info(
            f"distance remaining: {dist:.2f} m", throttle_duration_sec=2.0
        )

    def _on_result(self, future) -> None:
        """Action callback: called when Nav2 reports that navigation has finished."""
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("reached goal")
        else:
            self.get_logger().warn(f"navigation ended with status {result.status}")
        self._navigating = False
        self._goal_handle = None

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._linear_x = msg.linear.x

    def _on_scan(self, msg: LaserScan) -> None:
        """Subscription callback: cancel navigation if an obstacle is critically close.

        Nav2's local costmap handles normal obstacle avoidance. This fires a hard cancel
        for cases where costmap updates haven't propagated in time — a last resort, not
        the primary safety mechanism. It only looks in the direction the robot is
        driving, so a robot parked near something can still be sent away from it.
        """
        if not self._navigating or abs(self._linear_x) < 0.02:
            return
        center = self._p_front_angle + (0.0 if self._linear_x > 0 else 180.0)
        closest = float("inf")
        for i, r in enumerate(msg.ranges):
            if not (r > 0.0 and math.isfinite(r)):
                continue
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)
            if abs((angle - center + 180.0) % 360.0 - 180.0) <= self._p_cone:
                closest = min(closest, r)
        if closest < self._p_stop_dist:
            self.get_logger().warn(f"emergency stop: obstacle at {closest:.2f} m ahead")
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self._navigating = False


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        # spin() hands control to the ROS2 executor, which drives all subscriptions,
        # timers, and action callbacks until the node is shut down.
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
