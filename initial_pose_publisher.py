#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from math import sin, cos

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # ─── 1) Declare & read parameters ───────────────────────────────────
        self.declare_parameter('robot_ns', 'robot_0')
        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.ns  = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.x   = float(self.get_parameter('x').get_parameter_value().double_value)
        self.y   = float(self.get_parameter('y').get_parameter_value().double_value)
        self.yaw = float(self.get_parameter('yaw').get_parameter_value().double_value)

        # ─── 2) Publisher on "/<ns>/initialpose" ────────────────────────────
        topic = f'/{self.ns}/initialpose'
        self.pub = self.create_publisher(PoseWithCovarianceStamped, topic, 1)

        # ─── 3) TF2 Buffer & Listener ────────────────────────────────────────
        # We will repeatedly try to lookup “odom → base_link” until it exists.
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ─── 4) Poll every 0.1 s of sim‐time; once lookup_transform succeeds, we publish and exit ─
        self.timer = self.create_timer(0.1, self.check_and_publish)

        self.get_logger().info(
            f'[{self.ns}] Waiting for “odom→base_link” to appear … will publish to "{topic}" '
            'using that exact timestamp, to avoid “extrapolation into the future.”'
        )

    def check_and_publish(self):
        """
        Called every 0.1 s. Try to get the latest transform from "odom" → "base_link".
        As soon as it succeeds, grab trans.header.stamp (e.g. sim‐time ~77.301 s), build
        a PoseWithCovarianceStamped with that same stamp, publish once, and shut down.
        """
        try:
            # Try to look up the most recent odom→base_link. If this throws, keep waiting.
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            # It isn’t available yet—AMCL/robot_state_publisher haven’t published it. Return and wait.
            return

        # ─── 5) As soon as we have a valid “odom→base_link” transform, grab its stamp ─────────
        stamp = trans.header.stamp  # This is a builtin rclpy.time.Time

        # Build the PoseWithCovarianceStamped at (x, y, yaw), using exactly that stamp:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = cos(self.yaw / 2.0)

        cov = [0.0] * 36
        cov[0]  = 0.25         # x variance
        cov[7]  = 0.25         # y variance
        cov[35] = 0.068538922  # yaw variance (~0.262 rad²)
        msg.pose.covariance = cov

        self.pub.publish(msg)

        sim_sec = stamp.sec + stamp.nanosec / 1e9
        self.get_logger().info(
            f'[{self.ns}] Published initial pose x={self.x}, y={self.y}, yaw={self.yaw} '
            f'at sim-time {sim_sec:.2f}s'
        )

        # Cancel our timer (so we only publish once), then shut down
        self.timer.cancel()
        rclpy.shutdown()

    def destroy_node(self):
        try:
            self.timer.cancel()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
