import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hybrid_dir = get_package_share_directory('hybrid_navigation')
    nav2_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    model_path = LaunchConfiguration('model_path')

    drl_inference_node = Node(
        package='td3_controller',
        executable='drl_nav2_inference.py',
        name='drl_nav2_inference_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_path': model_path,
            'state_dim': 44,
            'action_dim': 2,
            'max_action': 1.0,
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', description='Full path to map yaml'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(hybrid_dir, 'config', 'nav2_params_hybrid.yaml')),
        DeclareLaunchArgument(
            'model_path', default_value='models/actor_stage9_episode7400.pt',
            description='TD3 actor weights (relative to td3_controller share dir, or absolute)'),
        drl_inference_node,
        nav2_bringup,
    ])
