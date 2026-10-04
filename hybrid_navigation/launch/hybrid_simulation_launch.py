import os
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# name -> (world, map, default spawn x, y). Worlds and maps are (package, path inside its share dir).
ENVIRONMENTS = {
    'static_simple': (
        ('turtlebot3_gazebo', 'worlds/turtlebot3_dqn_stage2.world'),
        ('hybrid_navigation', 'maps/static_simple.yaml'), (0.0, 0.0)),
    'static_complex': (
        ('turtlebot3_gazebo', 'worlds/turtlebot3_world.world'),
        ('nav2_bringup', 'maps/turtlebot3_world.yaml'), (-2.0, -0.5)),
    'dynamic_simple': (
        ('hybrid_navigation', 'worlds/dynamic_simple.world'),
        ('hybrid_navigation', 'maps/dynamic_simple.yaml'), (0.0, 0.0)),
    'dynamic_complex': (
        ('hybrid_navigation', 'worlds/dynamic_complex.world'),
        ('hybrid_navigation', 'maps/dynamic_complex.yaml'), (-2.0, -0.5)),
}


# Files the launch needs from the TurtleBot3 simulation package
TB3_REQUIRED_FILES = (
    'urdf/turtlebot3_burger.urdf',
    'models/turtlebot3_burger/model.sdf',
    'models/turtlebot3_dqn_world/model.sdf',
    'worlds/turtlebot3_dqn_stage2.world',
    'worlds/turtlebot3_world.world',
)


def find_turtlebot3_share():
    """Share directory of a complete turtlebot3_gazebo package.

    Several workspaces can provide a package called turtlebot3_gazebo (for example a DRL
    training fork with a different layout), so every prefix on the path is checked and the
    first one that has all the files this launch needs is used.
    """
    prefixes = [p for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
    for prefix in prefixes:
        candidate = os.path.join(prefix, 'share', 'turtlebot3_gazebo')
        if all(os.path.exists(os.path.join(candidate, f)) for f in TB3_REQUIRED_FILES):
            return candidate
    raise RuntimeError(
        'No complete turtlebot3_gazebo package found. It must provide: ' +
        ', '.join(TB3_REQUIRED_FILES) +
        '. Install it with: sudo apt install ros-humble-turtlebot3-gazebo')


def share(package, path):
    if package == 'turtlebot3_gazebo':
        return os.path.join(find_turtlebot3_share(), path)
    return os.path.join(get_package_share_directory(package), path)


def params_with_initial_pose(x, y):
    """Copy of the hybrid Nav2 params in which AMCL starts at the robot's spawn pose."""
    with open(share('hybrid_navigation', 'config/nav2_params_hybrid.yaml')) as f:
        params = yaml.safe_load(f)
    amcl = params['amcl']['ros__parameters']
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': float(x), 'y': float(y), 'z': 0.0, 'yaw': 0.0}
    handle, path = tempfile.mkstemp(prefix='nav2_params_hybrid_', suffix='.yaml')
    with os.fdopen(handle, 'w') as f:
        yaml.safe_dump(params, f)
    return path


def launch_setup(context):
    name = LaunchConfiguration('environment').perform(context)
    if name not in ENVIRONMENTS:
        raise RuntimeError("Unknown environment '%s'. Choose one of: %s" % (
            name, ', '.join(ENVIRONMENTS)))
    (world_pkg, world_path), (map_pkg, map_path), default_xy = ENVIRONMENTS[name]
    world = share(world_pkg, world_path)
    map_file = share(map_pkg, map_path)

    x_arg = LaunchConfiguration('x_pose').perform(context)
    y_arg = LaunchConfiguration('y_pose').perform(context)
    x = float(x_arg) if x_arg else default_xy[0]
    y = float(y_arg) if y_arg else default_xy[1]

    tb3_share = find_turtlebot3_share()
    model_path = ':'.join(filter(None, [
        os.path.join(tb3_share, 'models'), os.environ.get('GAZEBO_MODEL_PATH', '')]))
    plugin_path = ':'.join(filter(None, [
        os.path.join(get_package_prefix('hybrid_navigation'), 'lib', 'hybrid_navigation'),
        os.environ.get('GAZEBO_PLUGIN_PATH', '')]))

    with open(os.path.join(tb3_share, 'urdf', 'turtlebot3_burger.urdf')) as f:
        robot_description = f.read()

    headless = LaunchConfiguration('headless')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    return [
        # Do not wait for the online Gazebo model database (hangs when offline)
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=UnlessCondition(headless)),

        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', 'turtlebot3_burger',
                       '-file', os.path.join(tb3_share, 'models', 'turtlebot3_burger', 'model.sdf'),
                       '-x', str(x), '-y', str(y), '-z', '0.01']),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),

        Node(
            package='td3_controller', executable='drl_nav2_inference.py',
            name='drl_nav2_inference_node', output='screen',
            parameters=[{
                'use_sim_time': True,
                'model_path': LaunchConfiguration('model_path'),
                'state_dim': 44,
                'action_dim': 2,
                'max_action': 1.0,
            }]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'True',
                'params_file': params_with_initial_pose(x, y),
                'autostart': 'True',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'rviz_launch.py')),
            condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'environment', default_value='static_complex',
            description='Test environment: ' + ', '.join(ENVIRONMENTS)),
        DeclareLaunchArgument(
            'x_pose', default_value='', description='Spawn x (default depends on the environment)'),
        DeclareLaunchArgument(
            'y_pose', default_value='', description='Spawn y (default depends on the environment)'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='Run Gazebo without its window'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument(
            'model_path', default_value='models/actor_stage9_episode7400.pt',
            description='TD3 actor weights (relative to the td3_controller share dir, or absolute)'),
        OpaqueFunction(function=launch_setup),
    ])
