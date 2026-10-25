import math
from collections import deque
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
from motor_control_bridge.usb_ports import NANO_DRIVER, find_port

# Demand is tracked in slices; dispatch fires once one kind is this far behind,
# and the backlog is capped so a slow robot doesn't build up unbounded motion.
_MIN_DEMAND_SLICES = 0.5
_CHAIN_MIN_DEMAND_SLICES = 0.0  # a slice continuing the current chain may go out early
_MAX_DEMAND_SLICES = 2.0
_MAX_DEMAND_DT = 0.5  # s; longest gap integrated into demand
_MAX_PIPELINE = 2  # slices outstanding; the second lets the firmware blend into it
_POLL_TIMEOUT = 1.0  # s; give up on a poll response that never arrives


# ── types ──────────────────────────────────────────────────────────────────────


class IntentKind(Enum):
    STOP = auto()
    MOVE = auto()
    ROTATE = auto()


class Intent(NamedTuple):
    """The motion the bridge should be executing right now."""

    kind: IntentKind
    direction: int = 0  # firmware direction, -1 or 1; unused for STOP


@dataclass
class _Pending:
    """A queued slice the firmware holds (active or waiting)."""

    seq: int
    intent: Intent
    acked: bool = False  # intake ACK received


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
        self.declare_parameter("serial_driver", NANO_DRIVER)  # finds the Nano again after a USB drop
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("slice_distance", 0.05)
        self.declare_parameter("slice_angle", 0.1)
        self.declare_parameter("angular_threshold", 0.05)
        self.declare_parameter("linear_threshold", 0.02)
        self.declare_parameter("track_width", 0.15)
        self.declare_parameter("cmd_vel_timeout", 0.5)

        # Cache parameters — these are set at launch and won't change at runtime,
        # so reading them once avoids repeated get_parameter() calls in hot paths.
        self._p_slice_dist = self.get_parameter("slice_distance").value
        self._p_slice_angle = self.get_parameter("slice_angle").value
        self._p_ang_thresh = self.get_parameter("angular_threshold").value
        self._p_lin_thresh = self.get_parameter("linear_threshold").value
        self._p_track = self.get_parameter("track_width").value
        self._p_cmd_timeout = self.get_parameter("cmd_vel_timeout").value

        driver = self.get_parameter("serial_driver").value
        self._serial = MotorSerial(
            self.get_parameter("serial_port").value,
            self.get_parameter("baud_rate").value,
            find_port=lambda: find_port(driver),
        )
        self._generation = self._serial.generation
        self._link_up = True

        # Bridge state — all fields are only touched on the ROS2 executor thread.
        self._pending: deque[_Pending] = deque()  # slices the firmware holds, oldest first
        self._poll_seq: int | None = None
        self._poll_sent = self.get_clock().now()
        self._stop_seq: int | None = None
        self._twist: Twist | None = None  # latest non-zero /cmd_vel; None = stopped
        self._twist_stamp = self.get_clock().now()
        self._demand_stamp = None  # last time demand was integrated
        self._demand_move = 0.0  # signed slices of distance Nav2 has asked for
        self._demand_turn = 0.0  # signed slices of angle (CCW positive)
        self._motion: Intent | None = None  # last queued command; attributes x2 deltas
        self._last_x2: float | None = None  # firmware's cumulative signed displacement
        self._v_fwd = 0.0
        self._omega = 0.0

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
        self.create_timer(0.05, self._republish_odom)  # 20 Hz — keep odom TF fresh

    # ── serial responses ────────────────────────────────────────────────────────

    def _check_link(self) -> None:
        """Reset slice state when the firmware link drops or comes back.

        A reconnect resets the Arduino, so its queue is empty and its cumulative
        displacement is back at zero: forget everything outstanding and re-baseline x2.
        """
        if not self._serial.connected:
            if self._link_up:
                self.get_logger().error("firmware serial link lost; waiting for it to return")
                self._link_up = False
                self._reset_link_state()
            return
        if not self._link_up or self._serial.generation != self._generation:
            self.get_logger().warn("firmware serial link restored; the firmware restarted")
            self._link_up = True
            self._generation = self._serial.generation
            self._reset_link_state()

    def _reset_link_state(self) -> None:
        self._pending.clear()
        self._poll_seq = None
        self._stop_seq = None
        self._last_x2 = None

    def _drain_responses(self) -> None:
        """Timer callback: process all serial responses that arrived since last tick."""
        self._check_link()
        for resp in self._serial.drain():
            self._handle_response(resp)

    def _handle_response(self, resp) -> None:
        seq = resp.seq_id
        if seq == self._stop_seq:
            # STOP ACK carries no odometry; poll for the aborted slice's displacement.
            self._stop_seq = None
            self._send_poll()
            return
        if seq == self._poll_seq:
            self._poll_seq = None
            self._integrate_displacement(resp.x2)
            self._dispatch()
            return

        cmd = next((c for c in self._pending if c.seq == seq), None)
        if cmd is None:
            return  # stale: superseded by a STOP

        if resp.status == Status.REJECTED_FULL:
            self.get_logger().warn("firmware queue full; slice rejected")
            self._pending.remove(cmd)
            return
        if resp.status == Status.OK and not cmd.acked:
            # Intake ACK; the completion follows when the slice finishes. Queue the
            # next slice behind it so the firmware can blend the two.
            cmd.acked = True
            self._dispatch()
            return

        if resp.status == Status.TIMEOUT:
            self.get_logger().warn("slice timed out (stalled, or hit the backstop)")

        # Completion carries no odometry, so poll for the slice's displacement.
        self._pending.remove(cmd)
        if self._poll_seq is None:
            self._send_poll()
        self._dispatch()

    # ── motion dispatch ─────────────────────────────────────────────────────────

    def _clear_demand(self) -> None:
        self._demand_stamp = None
        self._demand_move = 0.0
        self._demand_turn = 0.0

    def _accrue_demand(self) -> None:
        """Add the distance and angle the Twist has asked for since the last call."""
        now = self.get_clock().now()
        if self._twist is None:
            self._clear_demand()
            return
        dt = 0.0
        if self._demand_stamp is not None:
            dt = min((now - self._demand_stamp).nanoseconds * 1e-9, _MAX_DEMAND_DT)
        self._demand_stamp = now
        self._demand_move += self._twist.linear.x * dt / self._p_slice_dist
        self._demand_turn += self._twist.angular.z * dt / self._p_slice_angle

        # Scale both down together so the move/turn ratio survives the cap.
        peak = max(abs(self._demand_move), abs(self._demand_turn))
        if peak > _MAX_DEMAND_SLICES:
            scale = _MAX_DEMAND_SLICES / peak
            self._demand_move *= scale
            self._demand_turn *= scale

    def _timed_out(self) -> bool:
        age = (self.get_clock().now() - self._twist_stamp).nanoseconds * 1e-9
        return age > self._p_cmd_timeout

    def _next_intent(self, chain: Intent | None) -> Intent:
        """Pick the next slice that best approximates the current velocity command.

        The firmware can only move or rotate, so a Twist that mixes both is split
        into alternating slices: whichever of distance and angle is further behind
        is paid out. With a chain, only a slice that continues it is returned (a
        different motion must wait until the chain drains), and nothing is paid. A
        continuing slice goes out early so it is queued before the last one finishes.
        """
        if self._twist is None or self._timed_out():
            self._clear_demand()
            return Intent(IntentKind.STOP)
        self._accrue_demand()

        # Credit is progress toward the commanded direction; it may go negative after
        # a slice overshoots the demand, so total motion tracks the Twist. Only the
        # axes the Twist actually uses compete.
        sm = math.copysign(1.0, self._twist.linear.x) if self._twist.linear.x else 0.0
        st = math.copysign(1.0, self._twist.angular.z) if self._twist.angular.z else 0.0
        candidates = []
        if sm:
            candidates.append((self._demand_move * sm, Intent(IntentKind.MOVE, int(sm))))
        if st:
            # Firmware ROTATE +1 turns the robot counter-clockwise (verified on the robot).
            candidates.append((self._demand_turn * st, Intent(IntentKind.ROTATE, int(st))))
        if not candidates:
            return Intent(IntentKind.STOP)
        credit, intent = max(candidates, key=lambda c: c[0])
        threshold = _MIN_DEMAND_SLICES if chain is None else _CHAIN_MIN_DEMAND_SLICES
        if credit < threshold:
            return Intent(IntentKind.STOP)
        if chain is not None and intent != chain:
            return Intent(IntentKind.STOP)

        if intent.kind == IntentKind.MOVE:
            self._demand_move -= sm
        else:
            self._demand_turn -= st
        return intent

    def _dispatch(self) -> None:
        """Queue the next slice for the current velocity command, if there is room."""
        if self._stop_seq is not None or len(self._pending) >= _MAX_PIPELINE:
            return
        chain = self._pending[-1].intent if self._pending else None
        if chain is None and self._poll_seq is not None:
            return  # settle odometry before starting a new chain
        intent = self._next_intent(chain)
        match intent.kind:
            case IntentKind.MOVE:
                seq = self._serial.send(
                    OP.QUEUE_MOVE_DIST, self._p_slice_dist, intent.direction
                )
            case IntentKind.ROTATE:
                seq = self._serial.send(
                    OP.QUEUE_ROTATE_ANGLE, self._p_slice_angle, intent.direction
                )
            case IntentKind.STOP:
                return  # nothing to send
        if seq is None:
            return  # link down
        self._motion = intent
        self._pending.append(_Pending(seq, intent))

    def _send_poll(self) -> None:
        self._poll_seq = self._serial.send(OP.POLL_DIST)  # None if the link is down
        self._poll_sent = self.get_clock().now()

    def _send_stop(self, reason: str) -> None:
        """Interrupt now. STOP is not queued: it halts the active slice and drops the
        queued ones, and the firmware sends no completion for them, only a STOP ACK."""
        self.get_logger().info(f"stop: {reason}")
        self._pending.clear()
        self._poll_seq = None  # a poll in flight is stale; its delta rides the next one
        self._stop_seq = self._serial.send(OP.STOP)

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Subscription callback: called by the executor whenever Nav2 sends a velocity.

        /cmd_vel carries a Twist — linear.x (m/s forward) and angular.z (rad/s rotation).
        Nav2 streams this at ~10-20 Hz; each message is the desired velocity right now,
        not a queued movement to complete.
        """
        stopped = (
            abs(msg.linear.x) <= self._p_lin_thresh
            and abs(msg.angular.z) <= self._p_ang_thresh
        )
        if stopped:
            self._twist = None
            self._clear_demand()
            if self._pending and self._stop_seq is None:
                self._send_stop("zero velocity")
            return

        # Components inside their dead zone are dropped so noise doesn't rotate us.
        twist = Twist()
        twist.linear.x = msg.linear.x if abs(msg.linear.x) > self._p_lin_thresh else 0.0
        twist.angular.z = msg.angular.z if abs(msg.angular.z) > self._p_ang_thresh else 0.0
        self._accrue_demand()  # demand up to now at the previous Twist
        self._twist = twist
        self._twist_stamp = self.get_clock().now()
        self._dispatch()

    def _idle_poll(self) -> None:
        """Timer callback: stop if /cmd_vel went quiet, else keep odometry flowing.

        Nav2 and SLAM Toolbox require a continuous /odom stream even when the robot
        is stationary. When no motion command is active, the firmware sends no
        spontaneous responses, so we poll explicitly to keep the topic alive.
        """
        if self._pending and self._stop_seq is None and (self._twist is None or self._timed_out()):
            self._send_stop("no /cmd_vel")
            return
        if self._poll_seq is not None:
            age = (self.get_clock().now() - self._poll_sent).nanoseconds * 1e-9
            if age > _POLL_TIMEOUT:
                self.get_logger().warn("poll response lost; recovering")
                self._poll_seq = None
        if not self._pending and self._poll_seq is None and self._stop_seq is None:
            self._send_poll()

    # ── odometry ────────────────────────────────────────────────────────────────

    def _integrate_displacement(self, x2: float) -> None:
        """Update the dead-reckoning pose from the firmware's cumulative displacement.

        x2 is wheel B's signed cumulative distance (m), signed by command direction.
        Encoder A is unusable, so B alone gives the slice's motion: forward distance
        for a MOVE, and the chassis angle (both wheels cover the same arc) for a
        ROTATE. The firmware only accumulates while a command is active, so idle
        polls yield a zero delta.
        """
        now = self.get_clock().now()
        dt = (now - self._last_odom_stamp).nanoseconds * 1e-9
        self._last_odom_stamp = now

        dx = 0.0 if self._last_x2 is None else x2 - self._last_x2
        self._last_x2 = x2

        dist = 0.0
        dtheta = 0.0
        if self._motion is not None:
            if self._motion.kind == IntentKind.MOVE:
                dist = dx
            elif self._motion.kind == IntentKind.ROTATE:
                dtheta = dx * 2.0 / self._p_track  # +x2 is counter-clockwise

        # Midpoint heading keeps the arc of a slice from skewing the pose.
        mid = self._pose.theta + dtheta / 2.0
        self._pose.x += dist * math.cos(mid)
        self._pose.y += dist * math.sin(mid)
        self._pose.theta += dtheta

        if dt > 0.0:
            self._v_fwd = dist / dt
            self._omega = dtheta / dt

    def _republish_odom(self) -> None:
        """Timer callback: republish the held pose so the odom TF never goes stale
        while a slice is running."""
        self._publish_odom(self._v_fwd, self._omega, self.get_clock().now())

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
