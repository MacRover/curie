import os
import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

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
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_servo",
            default_value="true",
            description="Start MoveIt Servo"
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    start_servo = LaunchConfiguration("start_servo")

    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="curie_arm_moveit_config")
        .robot_description(file_path="config/arm.urdf.xacro")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("curie_arm_moveit_config", "config/servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    low_pass_filter_coeff = {"butterworth_filter_coeff": 70.0}

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("curie_arm_moveit_config"), "config", "arm.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
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
            ("/arm_position_controller/commands", "/arm_controller/pos_commands"),
        ],
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

    heartbeat_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="curie_hw_control",
                        executable="sparkmax_heartbeat",
                        output="both",
                        parameters=[{
                            "use_vcan": use_mock_hardware
                        }]
                    )
                ]
            )
        )
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

    delay_arm_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_position_controller", "--inactive", "--controller-manager", "/controller_manager"],
                )
            ]
        )
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            low_pass_filter_coeff,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    enable_req_servo_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=servo_node,
            on_start=[
                ExecuteProcess(
                    cmd=["ros2", "service", "call", "/servo_node/start_servo", "std_srvs/srv/Trigger", "{}"]
                ),
                LogInfo(msg="Sent enable request to MoveIt Servo")
            ]
        ),
        condition=IfCondition(start_servo)
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
        servo_node,
        enable_req_servo_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_arm_position_controller_spawner,
        delay_vel_controller_spawner,
        delay_gripper_controller_spawner,
        delay_rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes)
