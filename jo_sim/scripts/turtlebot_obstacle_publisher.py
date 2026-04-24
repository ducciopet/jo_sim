#!/usr/bin/env python3
"""
Publishes turtlebot's pose as an ObstacleArray in Jo's odom frame.

Position and velocity: /turtlebot/world_pose (geometry_msgs/PoseArray,
  bridged from /model/turtlebot/pose gz.msgs.Pose_V published by
  gz-sim OdometryPublisher with odom_frame=world).
  That plugin reads entity WorldPose from the physics ECM, so position
  stops updating the moment the robot stops — no wheel-slip drift.

Velocity is computed as the finite difference of the ground-truth
world position, then rotated into the odom frame.

Transform:  world --[inv(jo_spawn)]--> odom
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Quaternion
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


class TurtlebotObstaclePublisher(Node):
    def __init__(self):
        super().__init__("turtlebot_obstacle_publisher")

        self.declare_parameter("jo_spawn_x",   -6.5)
        self.declare_parameter("jo_spawn_y",    2.5)
        self.declare_parameter("jo_spawn_yaw",  -0.25)

        self._pub = self.create_publisher(
            ObstacleArray, "/onboard_detector/tracked_dynamic_obstacles", 10)

        self.create_subscription(
            PoseArray, "/turtlebot/world_pose", self._cb, 10)

        # state for finite-difference velocity and acceleration
        self._prev_wx:   float | None = None
        self._prev_wy:   float | None = None
        self._prev_vx_odom = 0.0
        self._prev_vy_odom = 0.0
        self._prev_time = None

    def _cb(self, msg: PoseArray):
        if not msg.poses:
            return

        now = self.get_clock().now()

        jo_sx   = self.get_parameter("jo_spawn_x").value
        jo_sy   = self.get_parameter("jo_spawn_y").value
        jo_syaw = self.get_parameter("jo_spawn_yaw").value

        pose = msg.poses[0]

        # ── position: world → odom ──────────────────────────────────────────
        wx   = pose.position.x
        wy   = pose.position.y
        wyaw = _yaw_from_quat(pose.orientation)

        dx = wx - jo_sx
        dy = wy - jo_sy
        cos_ji = math.cos(-jo_syaw)
        sin_ji = math.sin(-jo_syaw)
        odom_x   = dx * cos_ji - dy * sin_ji
        odom_y   = dx * sin_ji + dy * cos_ji
        odom_yaw = wyaw - jo_syaw

        # ── velocity: finite diff of world pos → odom frame ─────────────────
        vx_odom, vy_odom = 0.0, 0.0
        ax, ay = 0.0, 0.0

        if self._prev_wx is not None and self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds * 1e-9
            if dt > 0.0:
                vx_world = (wx - self._prev_wx) / dt
                vy_world = (wy - self._prev_wy) / dt
                # rotate world velocity into odom frame
                vx_odom = vx_world * cos_ji - vy_world * sin_ji
                vy_odom = vx_world * sin_ji + vy_world * cos_ji
                ax = (vx_odom - self._prev_vx_odom) / dt
                ay = (vy_odom - self._prev_vy_odom) / dt

        self._prev_wx = wx
        self._prev_wy = wy
        self._prev_vx_odom = vx_odom
        self._prev_vy_odom = vy_odom
        self._prev_time = now

        self.get_logger().info(
            f"turtlebot in odom: x={odom_x:.2f} y={odom_y:.2f} "
            f"yaw={math.degrees(odom_yaw):.1f}° "
            f"vx={vx_odom:.2f} vy={vy_odom:.2f}",
            throttle_duration_sec=1.0)

        stamp = now.to_msg()

        obs = Obstacle()
        obs.header.stamp = stamp
        obs.header.frame_id = "odom"
        obs.track_id = TRACK_ID
        obs.pose.position.x = odom_x
        obs.pose.position.y = odom_y
        obs.pose.position.z = 0.0
        obs.pose.orientation = _quat_from_yaw(odom_yaw)
        obs.size.x = TURTLEBOT_SIZE_X
        obs.size.y = TURTLEBOT_SIZE_Y
        obs.size.z = TURTLEBOT_SIZE_Z
        obs.twist.linear.x = vx_odom
        obs.twist.linear.y = vy_odom
        obs.accel.linear.x = ax
        obs.accel.linear.y = ay
        obs.confidence = 1.0
        obs.is_static = False
        obs.is_yolo = True

        arr = ObstacleArray()
        arr.header.stamp = stamp
        arr.header.frame_id = "odom"
        arr.obstacles = [obs]
        self._pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(TurtlebotObstaclePublisher())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
