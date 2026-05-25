from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declared_arguments = [

    ]

    base_container_node = ComposableNodeContainer(
        name="base_container_node",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
            ComposableNode(
                package="curie_base",
                plugin="base::Basestation",
                name="base_node",
            ),
        ]
    )

    nodes = [
        base_container_node
    ]
    return LaunchDescription(declared_arguments + nodes)