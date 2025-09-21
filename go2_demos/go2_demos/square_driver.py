#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_normalize(a):
    while a > math.pi:  a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.odom_topic    = self.declare_parameter('odom_topic', '/odom').value

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # ✅ include topic + QoS
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.pose_ready = False
        self.x0 = self.y0 = 0.0
        self.yaw0 = 0.0

        # square parameters
        self.side_len   = 1.0
        self.lin_speed  = 0.25
        self.ang_speed  = 0.6
        self.turn_angle = math.pi/2

        self.state = 'FORWARD'
        self.sides_done = 0

        self.get_logger().info(f"cmd_vel: {self.cmd_vel_topic}, odom: {self.odom_topic}")

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        if not self.pose_ready:
            self.x0, self.y0, self.yaw0 = self.x, self.y, self.yaw
            self.pose_ready = True

    def stop(self):
        self.pub.publish(Twist())

    def tick(self):
        if not self.pose_ready:
            return
        tw = Twist()
        if self.state == 'FORWARD':
            dist = math.hypot(self.x - self.x0, self.y - self.y0)
            if dist < self.side_len:
                tw.linear.x = self.lin_speed
            else:
                self.stop()
                self.state = 'TURN'
                self.yaw0 = self.yaw
        elif self.state == 'TURN':
            dpsi = angle_normalize(self.yaw - self.yaw0)
            if abs(dpsi) < self.turn_angle:
                tw.angular.z = self.ang_speed
            else:
                self.stop()
                self.sides_done += 1
                if self.sides_done >= 4:
                    self.get_logger().info("Square complete. Stopping.")
                    self.destroy_timer(self.timer)
                    self.stop()
                    return
                self.state = 'FORWARD'
                self.x0, self.y0 = self.x, self.y
        self.pub.publish(tw)

def main():
    rclpy.init()
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
