from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("Reduced_CAD_Model_For_Software", package_name="reduced_cad_model_for_software_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
