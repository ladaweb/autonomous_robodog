#!/usr/bin/env python3
"""
Move 1 m forward, then turn left 90°, then right 90°, then exit.
Params (via --ros-args -p):
  - cmd_vel_topic (str, default: /cmd_vel)
  - odom_topic    (str, default: /odom)
  - dist          (float, default: 1.0)   # forward distance (m)
  - lin_speed     (float, default: 0.2)   # m/s
  - ang_speed     (float, default: 0.6)   # rad/s
  - turn_deg      (float, default: 90.0)  # degrees
  - odom_reliable (bool,  default: False) # True if your /odom is RELIABLE
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def yaw_from_quat(q):
    """Extract yaw from geometry_msgs/Quaternion."""
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def angle_normalize(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MoveTurn(Node):
    def __init__(self):
        super().__init__('move_turn')

        # Parameters
        self.cmd = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.odom = self.declare_parameter('odom_topic', '/odom').value
        self.dist = float(self.declare_parameter('dist', 1.0).value)
        self.v = float(self.declare_parameter('lin_speed', 0.2).value)
        self.w = float(self.declare_parameter('ang_speed', 0.6).value)
        self.turn = math.radians(float(self.declare_parameter('turn_deg', 90.0).value))
        odom_reliable = bool(self.declare_parameter('odom_reliable', False).value)

        # Publisher / Subscriber
        self.pub = self.create_publisher(Twist, self.cmd, 10)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE if odom_reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # ✅ Correct arg order: (msg_type, topic_name, callback, qos)
        self.sub = self.create_subscription(Odometry, self.odom, self.odom_cb, qos)

        # State
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz
        self.done = Future()
        self.pose_ready = False
        self.x0 = self.y0 = 0.0
        self.yaw0 = 0.0
        self.state = 'FWD'
        self.turns_done = 0  # 0: left pending, 1: right pending, 2: done
        self.start_time = self.get_clock().now()

        self.get_logger().info(f"cmd_vel={self.cmd}, odom={self.odom}")

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        if not self.pose_ready:
            self.x0, self.y0, self.yaw0 = self.x, self.y, self.yaw
            self.pose_ready = True

    def stop(self):
        self.pub.publish(Twist())

    def finish(self, ok=True, msg="Done"):
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        self.stop()
        self.get_logger().info(msg)
        if not self.done.done():
            self.done.set_result(ok)

    def tick(self):
        # Timeout if no odom appears within 5 seconds
        if not self.pose_ready:
            if (self.get_clock().now() - self.start_time).nanoseconds > 5e9:
                self.finish(False, "No /odom received in 5s. Exiting.")
            return

        tw = Twist()

        if self.state == 'FWD':
            d = math.hypot(self.x - self.x0, self.y - self.y0)
            if d < self.dist:
                tw.linear.x = self.v
            else:
                self.stop()
                self.state = 'TURN_L'
                self.yaw0 = self.yaw

        elif self.state == 'TURN_L':
            dpsi = angle_normalize(self.yaw - self.yaw0)
            if abs(dpsi) < self.turn:
                tw.angular.z = self.w  # left (+z)
            else:
                self.stop()
                self.turns_done = 1
                self.state = 'TURN_R'
                self.yaw0 = self.yaw

        elif self.state == 'TURN_R':
            dpsi = angle_normalize(self.yaw - self.yaw0)
            if abs(dpsi) < self.turn:
                tw.angular.z = -self.w  # right (-z)
            else:
                self.stop()
                self.turns_done = 2
                self.finish(True, "Forward 1 m + left 90° + right 90° complete.")
                return

        self.pub.publish(tw)


def main():
    rclpy.init()
    node = MoveTurn()
    try:
        rclpy.spin_until_future_complete(node, node.done)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; stopping.")
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
