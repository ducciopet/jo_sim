import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    jo_sim_pkg = get_package_share_directory("jo_sim")
    jo_nav_pkg = get_package_share_directory("jo_navigation")

    default_world = os.path.join(
        jo_sim_pkg,
        "worlds",
        "external",
        "worlds",
        "office_cpr.world",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Shared Gazebo world for both robots",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch Nav2 RViz for jo",
    )
    glim_arg = DeclareLaunchArgument(
        "glim",
        default_value="false",
        description="Launch the GLIM stack for jo",
    )
    glim_param_arg = DeclareLaunchArgument(
        "glim_param",
        default_value=os.path.join(
            jo_sim_pkg,
            "config",
            "glim",
            "glim_config_bunker_sim",
        ),
        description="GLIM param folder passed to jo GLIM node",
    )
    localization_arg = DeclareLaunchArgument(
        "localization",
        default_value="true",
        description="Launch jo localization",
    )
    navigation_arg = DeclareLaunchArgument(
        "navigation",
        default_value="true",
        description="Launch jo local navigation",
    )
    teleop_turtlebot_arg = DeclareLaunchArgument(
        "teleop_turtlebot",
        default_value="false",
        description="Launch keyboard teleop for turtlebot on /turtlebot/cmd_vel",
    )

    jo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jo_sim_pkg, "launch", "launch_sim.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "rviz": "false",
            "glim": LaunchConfiguration("glim"),
            "glim_param": LaunchConfiguration("glim_param"),
        }.items(),
    )

    turtlebot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jo_sim_pkg, "launch", "launch_turtlebot_sim.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "rviz": "false",
            "teleop": LaunchConfiguration("teleop_turtlebot"),
            "start_gazebo": "false",
            "start_clock_bridge": "false",
        }.items(),
    )

    localization = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(jo_nav_pkg, "launch", "localization.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("localization")),
                launch_arguments={"use_sim_time": "true"}.items(),
            )
        ],
    )

    navigation = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(jo_nav_pkg, "launch", "navigation_local.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("navigation")),
                launch_arguments={
                    "use_sim_time": "true",
                    "rviz": "false",
                }.items(),
            )
        ],
    )

    rviz = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "sleep 3; rviz2 -d "
            + os.path.join(jo_nav_pkg, "rviz", "navigation.rviz"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            world_arg,
            rviz_arg,
            glim_arg,
            glim_param_arg,
            localization_arg,
            navigation_arg,
            teleop_turtlebot_arg,
            jo_sim,
            turtlebot_sim,
            localization,
            navigation,
            LogInfo(
                condition=IfCondition(LaunchConfiguration("rviz")),
                msg="Launching RViz for jo navigation",
            ),
            rviz,
        ]
    )
