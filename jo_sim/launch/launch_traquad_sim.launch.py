import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Whether to launch RViz"
    )
    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="false",
        description="Launch keyboard teleop publishing to /cmd_vel",
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
            get_package_share_directory("jo_sim"),
            "worlds",
            "external",
            "worlds",
            "office_cpr.world",
        ),
        description="World to load",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("jo_sim"), "rviz", "sim.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("mulinex_description"),
                    "urdf",
                    "traquad.xacro",
                ),
                " use_gazebo:=true",
                " enable_imu:=false",
                " enable_lidar:=false",
                " lidar_3D:=false",
                " mulinex_namespace:=traquad",
                " LF_HFE:=1.13",
                " LH_HFE:=-1.13",
                " RF_HFE:=-1.13",
                " RH_HFE:=1.13",
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                "frame_prefix": "traquad/",
            }
        ],
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
            "-topic",
            "robot_description",
            "-name",
            "traquad",
            "-x",
            "-2.0",
            "-y",
            "0.0",
            "-z",
            "1.0",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    wheel_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheels_vel_controller",
            "--param-file",
            os.path.join(
                get_package_share_directory("mulinex_ignition"),
                "config",
                "mulinex_mf.yaml",
            ),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster_1",
            "--param-file",
            os.path.join(
                get_package_share_directory("mulinex_ignition"),
                "config",
                "mulinex_mf.yaml",
            ),
        ],
        output="screen",
    )

    pd_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pd_controller",
            "--param-file",
            os.path.join(
                get_package_share_directory("mulinex_ignition"),
                "config",
                "mulinex_mf.yaml",
            ),
        ],
        output="screen",
    )

    cmd_bridge = Node(
        package="jo_sim",
        executable="traquad_cmd_bridge.py",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "cmd_vel_topic": "/traquad/cmd_vel",
            }
        ],
    )

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix=["xterm -e "],
        remappings=[("cmd_vel", "/traquad/cmd_vel")],
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
            robot_state_publisher,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_entity, on_exit=[pd_control_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_entity, on_exit=[wheel_control_spawner])
            ),
            cmd_bridge,
            teleop,
            rviz,
        ]
    )
