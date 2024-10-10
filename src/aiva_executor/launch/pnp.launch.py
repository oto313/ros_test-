from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("nova2_with_gripper")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # MTC Demo node
    pick_place_demo = Node(
        # package="mtc_tutorial",
        # executable="mtc_node",
        package="aiva_executor",
        executable="pick_place_demo",
        output="screen",
        parameters=[
                        moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            os.path.join(get_package_share_directory("aiva_executor"), "config", "config.yaml"),
        ],
    )

    return LaunchDescription([pick_place_demo])
