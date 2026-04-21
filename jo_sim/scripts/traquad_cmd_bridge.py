#!/usr/bin/env python3

import math

import rclpy
from custom_interfaces.msg import WheelVelocityCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TraquadCmdBridge(Node):
    def __init__(self) -> None:
        super().__init__("traquad_cmd_bridge")

        self.declare_parameter("wheel_speed_scale", 35.0)
        self.declare_parameter("yaw_scale", 0.35)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "output_topic", "/wheels_vel_controller/wheels_velocity_cmd"
        )

        self.wheel_speed_scale = float(
            self.get_parameter("wheel_speed_scale").value
        )
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.publisher = self.create_publisher(WheelVelocityCommand, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        turn = self.yaw_scale * wz
        scale = self.wheel_speed_scale

        wheel_cmd = WheelVelocityCommand()
        wheel_cmd.v_lf = scale * (-vx + vy - turn)
        wheel_cmd.v_lb = scale * (-vx - vy - turn)
        wheel_cmd.v_rf = scale * (vx + vy + turn)
        wheel_cmd.v_rb = scale * (vx - vy + turn)

        for field in ("v_lf", "v_lb", "v_rf", "v_rb"):
            value = getattr(wheel_cmd, field)
            if not math.isfinite(value):
                setattr(wheel_cmd, field, 0.0)

        self.publisher.publish(wheel_cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TraquadCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
