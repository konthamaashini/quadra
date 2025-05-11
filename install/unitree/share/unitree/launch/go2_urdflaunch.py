import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def validate_urdf(context, *args, **kwargs):
    urdf_path = context.launch_configurations['urdf_path']
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    return []

def generate_launch_description():
    # Define package and file paths
    package_name = 'unitree'
    urdf_file = 'urdf/go2_urdf.urdf'

    # Paths
    pkg_share = get_package_share_directory(package_name)
    default_urdf_path = os.path.join(pkg_share, urdf_file)
    world_file_path = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')

    # Declare LaunchConfiguration for urdf_path
    urdf_path = LaunchConfiguration('urdf_path')

    # Process the URDF file with xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Joint State Publisher Node (with GUI for manual joint control)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': True, 'use_sim_time': True}]
    )

    # Include Gazebo Classic
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            'extra_gazebo_args': '--verbose'
        }.items()
    )

    # Spawn entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'go2',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # Suppress ALSA warnings
    suppress_alsa_warnings = SetEnvironmentVariable(
        name='ALSA_CONFIG_PATH',
        value='/etc/alsa/alsa.conf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf_path',
            default_value=default_urdf_path,
            description='Absolute path to robot URDF file'
        ),
        OpaqueFunction(function=validate_urdf),
        suppress_alsa_warnings,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_launch,
        spawn_entity_node
    ])
