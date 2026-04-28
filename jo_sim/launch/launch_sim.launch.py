import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node

sys.path.insert(0, os.path.join(get_package_share_directory('jo_sim'), 'scripts'))
from launch_utils import inject_plugins  # noqa: E402


def generate_launch_description():

    package_name = 'jo_sim'

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Whether to launch RViz')

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'sim.rviz'
    )

    glim_config = os.path.join(get_package_share_directory(package_name), 'config', 'glim', 'glim_config_bunker_sim')

    glim_param_folder_arg = DeclareLaunchArgument(
        'glim_param',
        default_value=glim_config,
        description='GLIM param folder passed to its node'
    )

    launch_glim_arg = DeclareLaunchArgument(
        'glim',
        default_value='false',
        description='Whether to launch the GLIM stack'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    rsp = GroupAction(
        actions=[
            PushRosNamespace('jo'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('jo_description'), 'launch', 'description.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
            )
        ]
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'external',
        'worlds',
        'office_cpr.world'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load (plugins are injected automatically)'
    )

    inject = OpaqueFunction(function=inject_plugins)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v3 ', LaunchConfiguration('world')], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/jo/robot_description',
                                   '-name', 'jo',
                                   '-x', '-6.5',
                                   '-y', '2.5',
                                   '-z', '1.7',
                                   '-R', '0.0',
                                   '-P', '0.0',
                                   '-Y', '-0.25'],
                        output='screen')

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/front_camera/image", "/back_camera/image"],
    )

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
            {'use_sim_time': True}
        ],
    )

    delayed_glim = TimerAction(
        period=4.0,
        actions=[glim]
    )

    return LaunchDescription([
        glim_param_folder_arg,
        launch_glim_arg,
        rsp,
        rviz_arg,
        world_arg,
        inject,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        rviz,
        delayed_glim
    ])
