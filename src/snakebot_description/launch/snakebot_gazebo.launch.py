"""Spawn the robot, activate controllers in order, then enable its wave."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
import xacro
import yaml


def after_success(actions, label):
    def callback(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'{label} failed (exit {event.returncode}); stopping bringup'))]
        return actions
    return callback


def launch_setup(context):
    share = Path(get_package_share_directory('snakebot_description'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    composition = LaunchConfiguration('use_composition').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    world = str(share / 'worlds' / 'empty.sdf')
    controllers_yaml = os.path.join(
        get_package_share_directory('snakebot_description'),
        'config', 'controllers.yaml')
    bridge_config = str(share / 'config' / 'ros_gz_bridge.yaml')
    gui_config = str(share / 'config' / 'gui.config')
    forest_map = str(share / 'config' / 'forest_map.yaml')
    ground_height = float(yaml.safe_load(Path(forest_map).read_text())['ground_height'])
    description = xacro.process_file(
        str(share / 'urdf' / 'snakebot.urdf.xacro'),
        mappings={'controllers_file': controllers_yaml}).toxml()

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': description}],
        output='screen')
    spawn_entity = Node(
        package='ros_gz_sim', executable='create', name='spawn_snakebot',
        arguments=['-world', 'snake_world', '-name', 'snakebot',
                   '-topic', '/robot_description', '-z', str(ground_height+0.060),
                   '-x', LaunchConfiguration('spawn_x'),
                   '-y', LaunchConfiguration('spawn_y'),
                   '-Y', LaunchConfiguration('spawn_yaw'),
                   '-allow_renaming', 'false'],
        parameters=[{'use_sim_time': True}], output='screen')
    broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                   '--param-file', controllers_yaml,
                   '--controller-manager-timeout', '60', '--switch-timeout', '30'],
        parameters=[{'use_sim_time': True}], output='screen')
    trajectory_spawner = Node(
        package='controller_manager', executable='spawner',
        name='joint_trajectory_controller_spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager',
                   '--param-file', controllers_yaml,
                   '--controller-manager-timeout', '60', '--switch-timeout', '30'],
        parameters=[{'use_sim_time': True}], output='screen')
    behavior = Node(
        package='snakebot_description', executable='snake_behavior.py',
        prefix='/data/ros2_ws/.venv/bin/python3', name='snake_behavior',
        parameters=[{'use_sim_time': True, 'map_file': forest_map,
                     'autonomous': LaunchConfiguration('autonomous')}], output='screen')

    # Register BEFORE starting target processes. Exit codes must also be zero;
    # OnProcessExit alone would otherwise advance after a failed spawn.
    actions = [
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_entity,
            on_exit=after_success([broadcaster_spawner], 'Entity spawn'))),
        RegisterEventHandler(OnProcessExit(
            target_action=broadcaster_spawner,
            on_exit=after_success([trajectory_spawner], 'Joint state broadcaster'))),
        RegisterEventHandler(OnProcessExit(
            target_action=trajectory_spawner,
            on_exit=after_success([behavior], 'Joint trajectory controller'))),
    ]
    if composition:
        # gz_sim.launch.py does not implement use_composition in ros_gz 3.0.10.
        # Use the official server launch plus a shared multithreaded container.
        actions.extend([
            ComposableNodeContainer(
                name='snakebot_container', namespace='', package='rclcpp_components',
                executable='component_container_mt', output='screen',
                parameters=[{'use_sim_time': True}]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_server.launch.py')),
                launch_arguments={
                    'world_sdf_file': world, 'use_composition': 'True',
                    'create_own_container': 'False', 'container_name': 'snakebot_container',
                    'verbosity_level': '3',
                }.items()),
            LoadComposableNodes(
                target_container='snakebot_container',
                composable_node_descriptions=[ComposableNode(
                    package='ros_gz_bridge', plugin='ros_gz_bridge::RosGzBridge',
                    name='parameter_bridge',
                    parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
                    extra_arguments=[{'use_intra_process_comms': True}])]),
        ])
        if gui:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_sim.launch.py')),
                launch_arguments={'gz_args': f'-g -v 3 --render-engine ogre2 --gui-config {gui_config}',
                                  'on_exit_shutdown': 'true'}.items()))
    else:
        gz_args = (f'-r -v 3 --render-engine ogre2 --gui-config {gui_config} {world}'
                   if gui else f'-r -s -v 3 --render-engine ogre2 {world}')
        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_sim.launch.py')),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='parameter_bridge',
                 parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
                 output='screen'),
        ])
    actions.extend([state_publisher, spawn_entity])
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('GZ_RENDER_ENGINE', 'ogre2'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('autonomous', default_value='true', description='Enable planar navigation'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Initial model X'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Initial model Y'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Initial model yaw'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='Share a server/bridge component container'),
        OpaqueFunction(function=launch_setup),
    ])
