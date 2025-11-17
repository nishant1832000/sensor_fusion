from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('sensor_fusion_mt')

    # Launch arguments
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'new_world.sdf'])
    robot_file = PathJoinSubstitution([pkg_share, 'models', 'robot.urdf.xacro'])

    robot_description = Command(['xacro ', robot_file])

    # Start Gazebo (Ignition)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )

    # Spawn robot inside Ignition using ros_gz bridge
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description,
            '-name', 'my_robot',
            '-allow-renaming', 'true'
        ]
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot
    ])
