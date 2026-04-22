import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    turtlebot_desc_pkg = get_package_share_directory("turtlebot_description")
    jo_sim_pkg = get_package_share_directory("jo_sim")

    urdf_file = os.path.join(turtlebot_desc_pkg, "urdf", "turtlebot.urdf")
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Whether to launch RViz"
    )
    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="false",
        description="Launch keyboard teleop publishing to /turtlebot/cmd_vel",
    )
    start_gazebo_arg = DeclareLaunchArgument(
        "start_gazebo",
        default_value="true",
        description="Launch a new Gazebo instance",
    )
    start_clock_bridge_arg = DeclareLaunchArgument(
        "start_clock_bridge",
        default_value="true",
        description="Launch the /clock bridge",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            jo_sim_pkg,
            "worlds",
            "external",
            "worlds",
            "office_cpr.world",
        ),
        description="World to load",
    )

    rsp = GroupAction(
        actions=[
            PushRosNamespace("turtlebot"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": True,
                        "frame_prefix": "turtlebot/",
                    }
                ],
            ),
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_gazebo")),
        launch_arguments={
            "gz_args": ["-r -v3 ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_clock_bridge")),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/turtlebot/robot_description",
            "-name", "turtlebot",
            "-x", "2.5",
            "-y", "6.0",
            "-z", "0.1",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "-2.0"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    bridge_params = os.path.join(jo_sim_pkg, "config", "turtlebot_gz_bridge.yaml")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    obstacle_publisher = Node(
        package="jo_sim",
        executable="turtlebot_obstacle_publisher.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix=["xterm -e "],
        remappings=[("cmd_vel", "/turtlebot/cmd_vel")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    return LaunchDescription(
        [
            rviz_arg,
            teleop_arg,
            start_gazebo_arg,
            start_clock_bridge_arg,
            world_arg,
            gazebo,
            clock_bridge,
            rsp,
            spawn_entity,
            bridge,
            obstacle_publisher,
            teleop,
        ]
    )
