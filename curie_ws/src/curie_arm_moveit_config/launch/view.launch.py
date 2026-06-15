import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF path
    urdf_path = os.path.join(get_package_share_directory('curie_robot_description'), 'arm', 'urdf', 'arm.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'arm'],
                        output='screen')

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])