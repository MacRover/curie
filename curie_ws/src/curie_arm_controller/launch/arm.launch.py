from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware for simulation and testing.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("curie_arm_moveit_config"), "config", "arm.urdf.xacro"]
            ),
            " ",
            "use_vcan:=",
            use_mock_hardware,
        ],
        on_stderr="ignore"
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("curie_arm_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("curie_robot_description"), "arm/rviz", "arm.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/arm_velocity_controller/commands", "/arm_controller/vel_commands"),
        ],
    )

    heartbeat_node = Node(
        package="curie_hw_control",
        executable="sparkmax_heartbeat",
        output="both",
        parameters=[{
            "use_vcan": use_mock_hardware
        }]
    )

    spark_mock_handler = ExecuteProcess(
        cmd=[
            "spark_mock_runner.py", 
            PathJoinSubstitution(
                [FindPackageShare("curie_arm_controller"), "config", "arm_runner.yaml"]
            )
        ],
        name="spark_mock_runner",
        output="screen",
        condition=IfCondition(use_mock_hardware)
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                )
            ]
        )
    )

    delay_vel_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_velocity_controller", "--controller-manager", "/controller_manager"],
                )
            ]
        )
    )

    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["hand_controller", "--controller-manager", "/controller_manager"],
                )
            ]
        )
    )

    delay_rviz_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        robot_state_publisher_node,
        control_node,
        heartbeat_node,
        spark_mock_handler,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_vel_controller_spawner,
        # delay_gripper_controller_spawner,
        delay_rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes)
