from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription 


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arm", package_name="curie_arm_moveit_config").to_moveit_configs()
    
    moveit_testing_node = Node( 
        package='moveit_testing' , 
        executable='moveit_testing' ,
        name='moveit_testing' ,
        parameters=[moveit_config.to_dict()],
    )
    
    demo = generate_demo_launch(moveit_config)
    demo.add_action(moveit_testing_node)
    return demo
