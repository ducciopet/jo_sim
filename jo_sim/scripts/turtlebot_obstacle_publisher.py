#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from jo_msgs.msg import Obstacle, ObstacleArray

TURTLEBOT_SIZE_X = 0.6
TURTLEBOT_SIZE_Y = 0.6
TURTLEBOT_SIZE_Z = 1.1
TRACK_ID = 1


class TurtlebotObstaclePublisher(Node):
    def __init__(self):
        super().__init__("turtlebot_obstacle_publisher")
        self._pub = self.create_publisher(ObstacleArray, "/turtlebot/obstacle", 10)
        self.create_subscription(Odometry, "/turtlebot/odom", self._odom_cb, 10)
        self._prev_twist = None
        self._prev_time = None

    def _odom_cb(self, msg: Odometry):
        now = self.get_clock().now()

        obs = Obstacle()
        obs.header.stamp = msg.header.stamp
        obs.header.frame_id = "map"
        obs.track_id = TRACK_ID
        obs.pose = msg.pose.pose
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
        array.header.frame_id = "map"
        array.obstacles = [obs]

        self._pub.publish(array)


def main():
    rclpy.init()
    rclpy.spin(TurtlebotObstaclePublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
