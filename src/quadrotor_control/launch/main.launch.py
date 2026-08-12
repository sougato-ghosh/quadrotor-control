import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_quadrotor_control = get_package_share_directory('quadrotor_control')

    # Path to world SDF
    # Note: We can mount or copy the world.sdf inside our workspace, e.g. at /workspace/world.sdf
    world_path = os.path.join(pkg_quadrotor_control, 'world.sdf')
    if not os.path.exists(world_path):
        world_path = '/workspace/world.sdf'

    # 1. Start Ignition Gazebo Sim
    # Use '--render-engine ogre' to force Ogre 1.x which resolves black screen issues under software rendering or Docker GPU acceleration
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-r', '--render-engine', 'ogre'],
        output='screen'
    )

    # 2. Start ros_gz_bridge (parameter bridge) with our config file
    config_file = os.path.join(pkg_quadrotor_control, 'config', 'bridge_config.yaml')
    start_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': config_file}],
        output='screen'
    )

    # 3. Start our custom GUI / control node
    start_control_node = Node(
        package='quadrotor_control',
        executable='control_node',
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        start_bridge,
        start_control_node
    ])
