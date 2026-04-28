import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

sys.path.insert(0, os.path.join(get_package_share_directory('jo_sim'), 'scripts'))
from launch_utils import inject_plugins  # noqa: E402


def generate_launch_description():
    package_name = 'jo_sim'
    jo_sim_pkg = get_package_share_directory(package_name)
    turtlebot_desc_pkg = get_package_share_directory('turtlebot_description')

    with open(os.path.join(turtlebot_desc_pkg, 'urdf', 'turtlebot.urdf'), 'r') as f:
        turtlebot_description = f.read()

    glim_config = os.path.join(jo_sim_pkg, 'config', 'glim', 'glim_config_bunker_sim')

    # ── args ────────────────────────────────────────────────────────────────
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description='Whether to launch RViz')
    glim_param_folder_arg = DeclareLaunchArgument(
        'glim_param', default_value=glim_config, description='GLIM param folder')
    launch_glim_arg = DeclareLaunchArgument(
        'glim', default_value='false', description='Whether to launch the GLIM stack')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(jo_sim_pkg, 'worlds', 'external', 'worlds', 'office_cpr.world'),
        description='World to load (plugins are injected automatically)')
    teleop_arg = DeclareLaunchArgument(
        'teleop_turtlebot', default_value='false',
        description='Launch keyboard teleop for turtlebot')

    inject = OpaqueFunction(function=inject_plugins)

    # ── Jo RSP ──────────────────────────────────────────────────────────────
    rsp_jo = GroupAction([
        PushRosNamespace('jo'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('jo_description'), 'launch', 'description.launch.py'
            )),
            launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items(),
        ),
    ])

    # ── Turtlebot RSP ────────────────────────────────────────────────────────
    rsp_turtlebot = GroupAction([
        PushRosNamespace('turtlebot'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': turtlebot_description,
                'use_sim_time': True,
                'frame_prefix': 'turtlebot/',
            }],
            # Publish to /turtlebot/tf instead of global /tf so that Jo's
            # ekf_node and glim_rosnode don't receive stale turtlebot wheel
            # transforms (bridge latency causes TF_OLD_DATA warnings).
            remappings=[
                ('/tf',        '/turtlebot/tf'),
                ('/tf_static', '/turtlebot/tf_static'),
            ],
        ),
    ])

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={
            'gz_args': ['-r -v3 ', LaunchConfiguration('world')],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Spawn jo ─────────────────────────────────────────────────────────────
    spawn_jo = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', '/jo/robot_description',
            '-name', 'jo',
            '-x', '-6.5', '-y', '2.5', '-z', '1.7',
            '-R', '0.0', '-P', '0.0', '-Y', '-0.25',
        ],
        output='screen',
    )

    # ── Spawn turtlebot ──────────────────────────────────────────────────────
    spawn_turtlebot = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', '/turtlebot/robot_description',
            '-name', 'turtlebot',
            '-x', '2.5', '-y', '6.0', '-z', '0.1',
            '-R', '0.0', '-P', '0.0', '-Y', '-2.0',
        ],
        output='screen',
    )

    # ── Jo bridges ───────────────────────────────────────────────────────────
    ros_gz_bridge_jo = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={os.path.join(jo_sim_pkg, "config", "gz_bridge.yaml")}'],
    )
    ros_gz_image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/front_camera/image', '/back_camera/image'],
    )

    # ── Turtlebot bridge ─────────────────────────────────────────────────────
    ros_gz_bridge_tb = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={os.path.join(jo_sim_pkg, "config", "turtlebot_gz_bridge.yaml")}'],
        parameters=[{'use_sim_time': True}],
    )

    # ── Obstacle publisher ────────────────────────────────────────────────────
    obstacle_publisher = Node(
        package='jo_sim',
        executable='turtlebot_obstacle_publisher.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'jo_spawn_x':   -6.5,
            'jo_spawn_y':    2.5,
            'jo_spawn_yaw':  -0.25,
            # chassis_height (0.273) + 0.1 from base_footprint_joint in robot_core.xacro
            'jo_base_link_height': 0.373,
        }],
    )

    # ── Teleop turtlebot ──────────────────────────────────────────────────────
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix=['xterm -e '],
        remappings=[('cmd_vel', '/turtlebot/cmd_vel')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('teleop_turtlebot')),
        additional_env={
            'DISPLAY': os.environ.get('DISPLAY', ':0'),
            'XAUTHORITY': os.environ.get('XAUTHORITY', '/tmp/.Xauthority'),
        },
    )

    # ── GLIM ─────────────────────────────────────────────────────────────────
    glim = Node(
        package='glim_ros',
        executable='glim_rosnode',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('glim')),
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '0',
            '__GLX_VENDOR_LIBRARY_NAME': '',
        },
        parameters=[
            {'config_path': LaunchConfiguration('glim_param')},
            {'use_sim_time': True},
        ],
    )
    delayed_glim = TimerAction(period=4.0, actions=[glim])

    # ── Static TFs ───────────────────────────────────────────────────────────
    # world → odom: Jo's spawn pose (GLIM initialises odom at Jo's spawn)
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'world', '--child-frame-id', 'odom',
            '--x', '-6.5', '--y', '2.5', '--z', '0.0',
            '--yaw', '-0.25', '--pitch', '0.0', '--roll', '0.0',
        ],
        parameters=[{'use_sim_time': True}],
    )
    # world → turtlebot/odom: turtlebot spawn pose (odom aligned with robot heading)
    static_tf_world_tb_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'world', '--child-frame-id', 'turtlebot/odom',
            '--x', '2.5', '--y', '6.0', '--z', '0.0',
            '--yaw', '-2.0', '--pitch', '0.0', '--roll', '0.0',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(jo_sim_pkg, 'rviz', 'sim.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        glim_param_folder_arg,
        launch_glim_arg,
        world_arg,
        teleop_arg,
        inject,
        rsp_jo,
        rsp_turtlebot,
        gazebo,
        spawn_jo,
        spawn_turtlebot,
        ros_gz_bridge_jo,
        ros_gz_image_bridge,
        ros_gz_bridge_tb,
        static_tf_world_odom,
        static_tf_world_tb_odom,
        obstacle_publisher,
        teleop,
        delayed_glim,
        rviz,
    ])
