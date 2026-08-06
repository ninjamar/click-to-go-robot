import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import NamedTuple

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from protocol import OP, Status
from rclpy.node import Node

from motor_control_bridge.serial_transport import MotorSerial

# ── types ──────────────────────────────────────────────────────────────────────


class BridgeState(Enum):
    IDLE = auto()
    IN_FLIGHT = auto()  # a command has been sent; waiting for the firmware's response


class IntentKind(Enum):
    STOP = auto()
    MOVE = auto()
    ROTATE = auto()


class Intent(NamedTuple):
    """The motion the bridge should be executing right now."""

    kind: IntentKind
    direction: int = 0  # -1 (backward/CW) or 1 (forward/CCW); unused for STOP


@dataclass
class OdomPose:
    """Accumulated dead-reckoning pose in the odom frame."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # heading in radians; 0 = forward along the +x axis


# ── node ───────────────────────────────────────────────────────────────────────


class BridgeNode(Node):
    def __init__(self):
        # Register this node with the ROS2 graph under the name "motor_control_bridge".
        # The name appears in `ros2 node list` and must match the YAML parameter file's
        # top-level key.
        super().__init__("motor_control_bridge")

        # Parameters are runtime-configurable values that can be set via CLI flags
        # (--ros-args -p name:=value) or a YAML file at launch.
        # declare_parameter registers a parameter with a default value.
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("slice_distance", 0.05)
        self.declare_parameter("slice_angle", 0.1)
        self.declare_parameter("angular_threshold", 0.05)
        self.declare_parameter("linear_threshold", 0.02)
        self.declare_parameter("track_width", 0.15)

        # Cache parameters — these are set at launch and won't change at runtime,
        # so reading them once avoids repeated get_parameter() calls in hot paths.
        self._p_slice_dist = self.get_parameter("slice_distance").value
        self._p_slice_angle = self.get_parameter("slice_angle").value
        self._p_ang_thresh = self.get_parameter("angular_threshold").value
        self._p_lin_thresh = self.get_parameter("linear_threshold").value
        self._p_track = self.get_parameter("track_width").value

        self._serial = MotorSerial(
            self.get_parameter("serial_port").value,
            self.get_parameter("baud_rate").value,
        )

        # Bridge state — all fields are only touched on the ROS2 executor thread.
        self._state = BridgeState.IDLE
        self._inflight_seq: int | None = None
        self._expect_intake = (
            False  # True after sending a queued command, until intake ACK
        )
        self._last_intent = Intent(IntentKind.STOP)

        # Pose accumulates from wheel encoder data (dead-reckoning).
        # SLAM Toolbox corrects the drift via the map→odom TF transform.
        self._pose = OdomPose()
        self._last_odom_stamp = self.get_clock().now()

        # A publisher writes messages to a named topic. Other nodes subscribed to
        # that topic receive each message. QoS depth 10 buffers up to 10 messages
        # if a subscriber is temporarily slow.
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # TF (Transform Frames) is a shared coordinate-frame tree. Instead of every
        # node tracking "where is the robot?" independently, they all read from this
        # tree. TransformBroadcaster publishes the odom→base_link transform each update.
        # base_link→base_laser is handled by the lidar launch file (ld14p.launch.py).
        self._tf_br = tf2_ros.TransformBroadcaster(self)

        # A subscription registers a callback that runs on the executor thread
        # whenever a message arrives on the named topic.
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # Timers fire their callback at a fixed rate on the executor thread.
        # The default executor is single-threaded, so callbacks never run concurrently.
        self.create_timer(
            0.05, self._drain_responses
        )  # 20 Hz — process serial responses
        self.create_timer(0.10, self._idle_poll)  # 10 Hz — keep odom live when stopped

    # ── serial responses ────────────────────────────────────────────────────────

    def _drain_responses(self) -> None:
        """Timer callback: process all serial responses that arrived since last tick."""
        for resp in self._serial.drain():
            self._handle_response(resp)

    def _handle_response(self, resp) -> None:
        if resp.seq_id != self._inflight_seq:
            return  # stale — seq_id was superseded by an intervening STOP

        if resp.status == Status.REJECTED_FULL:
            # Firmware queue was full; reset to idle so the next cmd_vel can retry.
            self._state = BridgeState.IDLE
            self._inflight_seq = None
            return

        if resp.status == Status.OK and self._expect_intake:
            # Intake ACK: the firmware accepted the queued command. A second response
            # (the completion) will follow when the command finishes executing.
            self._expect_intake = False
            return

        # Terminal response: a POLL result, STOP ACK, or queued command completion.
        # Always integrate — even zero velocities must publish to keep the odom TF
        # alive for SLAM Toolbox. Completion responses carry v1=v2=0 (firmware uses
        # sendStatus, not a full data response), so they momentarily report zero speed,
        # which is harmless since the next slice starts immediately after.
        self._integrate_odom(resp.v1, resp.v2)

        self._state = BridgeState.IDLE
        self._inflight_seq = None
        self._dispatch(self._last_intent)

    # ── motion dispatch ─────────────────────────────────────────────────────────

    def _compute_intent(self, twist: Twist) -> Intent:
        """Translate a continuous velocity command into a discrete motion intent.

        Because the firmware can only move or rotate (not both simultaneously),
        angular velocity takes priority over linear.
        """
        if abs(twist.angular.z) > self._p_ang_thresh:
            return Intent(IntentKind.ROTATE, 1 if twist.angular.z > 0 else -1)
        if abs(twist.linear.x) > self._p_lin_thresh:
            return Intent(IntentKind.MOVE, 1 if twist.linear.x > 0 else -1)
        return Intent(IntentKind.STOP)

    def _dispatch(self, intent: Intent) -> None:
        """Issue the next firmware command for the given intent, if the bridge is idle."""
        if self._state != BridgeState.IDLE:
            return
        match intent.kind:
            case IntentKind.MOVE:
                seq = self._serial.send(
                    OP.QUEUE_MOVE_DIST, self._p_slice_dist, intent.direction
                )
                self._arm_inflight(seq, queued=True)
            case IntentKind.ROTATE:
                seq = self._serial.send(
                    OP.QUEUE_ROTATE_ANGLE, self._p_slice_angle, intent.direction
                )
                self._arm_inflight(seq, queued=True)
            case IntentKind.STOP:
                pass  # already stopped; nothing to send

    def _arm_inflight(self, seq: int, queued: bool) -> None:
        """Mark a command as in-flight and configure whether to expect an intake ACK."""
        self._inflight_seq = seq
        self._expect_intake = queued
        self._state = BridgeState.IN_FLIGHT

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Subscription callback: called by the executor whenever Nav2 sends a velocity.

        /cmd_vel carries a Twist — linear.x (m/s forward) and angular.z (rad/s rotation).
        Nav2 streams this at ~10-20 Hz; each message is the desired velocity right now,
        not a queued movement to complete.
        """
        intent = self._compute_intent(msg)
        self._last_intent = intent

        if intent.kind == IntentKind.STOP and self._state == BridgeState.IN_FLIGHT:
            # Interrupt the active command immediately. STOP is not queued on the
            # firmware side — it executes instantly and does NOT send a completion
            # for the aborted command, only a STOP acknowledgement.
            seq = self._serial.send(OP.STOP)
            self._arm_inflight(seq, queued=False)
        elif self._state == BridgeState.IDLE:
            self._dispatch(intent)

    def _idle_poll(self) -> None:
        """Timer callback: request fresh odometry when the bridge is idle.

        Nav2 and SLAM Toolbox require a continuous /odom stream even when the robot
        is stationary. When no motion command is active, the firmware sends no
        spontaneous responses, so we poll explicitly to keep the topic alive.
        """
        if self._state == BridgeState.IDLE:
            seq = self._serial.send(OP.POLL_DIST)
            self._arm_inflight(seq, queued=False)

    # ── odometry ────────────────────────────────────────────────────────────────

    def _integrate_odom(self, v1: float, v2: float) -> None:
        """Update the dead-reckoning pose from the latest wheel velocities.

        v1/v2 are signed linear wheel velocities (m/s) from the firmware.
        Differential drive kinematics convert per-wheel values to robot-level
        forward velocity and angular rate, which are then integrated into a 2D pose.
        """
        now = self.get_clock().now()
        dt = (now - self._last_odom_stamp).nanoseconds * 1e-9
        self._last_odom_stamp = now
        v_fwd = (v1 + v2) / 2.0
        omega = (v2 - v1) / self._p_track

        self._pose.x += v_fwd * math.cos(self._pose.theta) * dt
        self._pose.y += v_fwd * math.sin(self._pose.theta) * dt
        self._pose.theta += omega * dt

        self._publish_odom(v_fwd, omega, now)

    def _publish_odom(self, v_fwd: float, omega: float, stamp) -> None:
        """Publish the current pose as both an Odometry message and a TF transform.

        Both carry the same data. /odom is for nodes that subscribe to it directly.
        The TF broadcast makes the pose available to any node that queries the
        coordinate tree — SLAM Toolbox reads odom→base_link to track the robot
        between scan-match corrections, then publishes its own map→odom correction.
        """
        # ROS2 rotation is expressed as a quaternion. For a ground robot that only
        # rotates around the vertical Z axis, only qz and qw are needed (qx, qy = 0).
        qz = math.sin(self._pose.theta / 2.0)
        qw = math.cos(self._pose.theta / 2.0)
        stamp_msg = stamp.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = "odom"  # the pose is expressed in the odom frame
        odom.child_frame_id = (
            "base_link"  # this describes where base_link is within odom
        )
        odom.pose.pose.position.x = self._pose.x
        odom.pose.pose.position.y = self._pose.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_fwd
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp_msg
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self._pose.x
        tf.transform.translation.y = self._pose.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_br.sendTransform(tf)


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        # spin() hands control to the ROS2 executor, which drives all timers and
        # subscription callbacks until the node is shut down (Ctrl-C or ros2 node kill).
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
