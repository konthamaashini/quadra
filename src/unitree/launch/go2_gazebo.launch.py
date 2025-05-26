import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the package directory
    pkg_unitree = get_package_share_directory('unitree')

    # Declare launch arguments
    urdf_file = os.path.join(pkg_unitree, 'urdf', 'go2_description.urdf')
    controllers_file = os.path.join(pkg_unitree, 'config', 'controllers1.yaml')

    # Load the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),

        # Spawn the robot in Gazebo
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'go2_description',
                 '-file', urdf_file,
                 '-x', '0.0', '-y', '0.0', '-z', '0.5'],
            output='screen'
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gzserver', '--minimal'],
            output='screen'
        ),

        # Load joint state broadcaster
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_desc}, controllers_file],
            output='screen'
        ),

        # Spawn joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Spawn position controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])