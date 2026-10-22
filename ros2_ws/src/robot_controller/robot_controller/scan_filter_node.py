import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    """Republishes /scan as /scan_filtered with the robot's own body removed.

    SLAM Toolbox has no minimum range, so the lidar's view of the wheels would
    otherwise end up in the map. Filtered returns become NaN, which Nav2 and
    SLAM treat as invalid rather than as free space.
    """

    def __init__(self):
        super().__init__("scan_filter")

        self.declare_parameter("scan_min_range", 0.27)
        self._p_min_range = self.get_parameter("scan_min_range").value

        # Reliable so both reliable and best-effort subscribers can connect.
        self._pub = self.create_publisher(
            LaserScan, "/scan_filtered", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg: LaserScan) -> None:
        msg.ranges = [math.nan if r < self._p_min_range else r for r in msg.ranges]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
