#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from jo_msgs.msg import Obstacle, ObstacleArray

TURTLEBOT_SIZE_X = 0.6
TURTLEBOT_SIZE_Y = 0.6
TURTLEBOT_SIZE_Z = 1.1
TRACK_ID = 1


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _world_from_odom(sx, sy, syaw, ox, oy, oq):
    """Compose spawn offset (sx, sy, syaw) with odom-relative displacement."""
    oyaw = _yaw_from_quat(oq)
    wx = sx + ox * math.cos(syaw) - oy * math.sin(syaw)
    wy = sy + ox * math.sin(syaw) + oy * math.cos(syaw)
    return wx, wy, _quat_from_yaw(syaw + oyaw)


class TurtlebotObstaclePublisher(Node):
    def __init__(self):
        super().__init__("turtlebot_obstacle_publisher")
        self.declare_parameter("spawn_x", 2.5)
        self.declare_parameter("spawn_y", 6.0)
        self.declare_parameter("spawn_yaw", -2.0)
        self._pub = self.create_publisher(ObstacleArray, "/onboard_detector/tracked_dynamic_obstacles", 10)
        self.create_subscription(Odometry, "/turtlebot/odom", self._odom_cb, 10)
        self._prev_twist = None
        self._prev_time = None

    def _odom_cb(self, msg: Odometry):
        now = self.get_clock().now()

        sx = self.get_parameter("spawn_x").value
        sy = self.get_parameter("spawn_y").value
        syaw = self.get_parameter("spawn_yaw").value

        op = msg.pose.pose
        wx, wy, wq = _world_from_odom(sx, sy, syaw, op.position.x, op.position.y, op.orientation)

        obs = Obstacle()
        obs.header.stamp = msg.header.stamp
        obs.header.frame_id = "odom"
        obs.track_id = TRACK_ID
        obs.pose.position.x = wx
        obs.pose.position.y = wy
        obs.pose.position.z = op.position.z
        obs.pose.orientation = wq
        obs.size.x = TURTLEBOT_SIZE_X
        obs.size.y = TURTLEBOT_SIZE_Y
        obs.size.z = TURTLEBOT_SIZE_Z
        obs.twist = msg.twist.twist

        if self._prev_twist is not None and self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds * 1e-9
            if dt > 0.0:
                obs.accel.linear.x = (msg.twist.twist.linear.x - self._prev_twist.linear.x) / dt
                obs.accel.linear.y = (msg.twist.twist.linear.y - self._prev_twist.linear.y) / dt

        obs.confidence = 1.0
        obs.is_static = False
        obs.is_yolo = True

        self._prev_twist = msg.twist.twist
        self._prev_time = now

        array = ObstacleArray()
        array.header.stamp = msg.header.stamp
        array.header.frame_id = "odom"
        array.obstacles = [obs]

        self._pub.publish(array)


def main():
    rclpy.init()
    rclpy.spin(TurtlebotObstaclePublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
