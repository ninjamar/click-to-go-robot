import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
_STATUS_NAMES = {
    v: k.removeprefix("STATUS_")
    for k, v in vars(GoalStatus).items()
    if k.startswith("STATUS_")
}


class ControllerNode(Node):
    """Safety monitor for Nav2 goals.

    Goals come from /goal_pose, which bt_navigator subscribes to itself. This node
    watches the NavigateToPose action, so it covers a goal from any source, and
    cancels it if the robot is about to hit something.
    """

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

        self._goals: dict[bytes, int] = {}  # goal uuid -> last seen status
        self._linear_x = 0.0  # latest commanded forward speed
        self._last_cancel = 0.0

        # An empty CancelGoal request cancels every goal the action server holds.
        self._cancel_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )

        # Action servers publish goal status latched, so match that to see current goals.
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_status,
            status_qos,
        )
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._on_feedback,
            10,
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # /scan_filtered is the lidar output — an array of distance readings at evenly-spaced
        # angles around the robot. Used here only as a last-resort safety check;
        # Nav2's local costmap handles normal obstacle avoidance.
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan, 10)

    @property
    def _navigating(self) -> bool:
        return any(status in _ACTIVE for status in self._goals.values())

    def _on_status(self, msg: GoalStatusArray) -> None:
        """Subscription callback: log goal state changes, whoever sent the goal."""
        for entry in msg.status_list:
            uuid = bytes(entry.goal_info.goal_id.uuid)
            if self._goals.get(uuid) == entry.status:
                continue
            self._goals[uuid] = entry.status
            self.get_logger().info(f"goal {uuid.hex()[:8]}: {_STATUS_NAMES[entry.status]}")

    def _on_feedback(self, msg) -> None:
        # throttle_duration_sec prevents log spam — at most one log line every 2 seconds.
        self.get_logger().info(
            f"distance remaining: {msg.feedback.distance_remaining:.2f} m",
            throttle_duration_sec=2.0,
        )

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
        if closest >= self._p_stop_dist or time.monotonic() - self._last_cancel < 1.0:
            return

        self.get_logger().warn(f"emergency stop: obstacle at {closest:.2f} m ahead")
        self._last_cancel = time.monotonic()
        if self._cancel_client.service_is_ready():
            self._cancel_client.call_async(CancelGoal.Request())
        else:
            self.get_logger().error("Nav2 cancel service unavailable")


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
