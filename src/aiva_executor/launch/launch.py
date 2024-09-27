#!/usr/bin/env -S ros2 launch
"""Launch C++ example for following a target"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    # declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    # description_package = LaunchConfiguration("description_package")
    # description_filepath = LaunchConfiguration("description_filepath")
    # moveit_config_package = "nova2_with_gripper"
    # robot_type = LaunchConfiguration("robot_type")
    # rviz_config = LaunchConfiguration("rviz_config")
    # use_sim_time = LaunchConfiguration("use_sim_time")
    # ign_verbosity = LaunchConfiguration("ign_verbosity")
    # log_level = LaunchConfiguration("log_level")

    # URDF
    # urdf_file_name = 'nova2_robot_with_pgc-50-35.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('dobot_rviz'),
    #     urdf_file_name)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()


    # List of included launch descriptions
    # launch_descriptions = [
    #     # Launch world with robot (configured for this example)
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("ign_moveit2_examples"),
    #                     "launch",
    #                     "default.launch.py",
    #                 ]
    #             )
    #         ),
    #         launch_arguments=[
    #             ("world_type", "follow_target"),
    #             ("robot_type", robot_type),
    #             ("rviz_config", rviz_config),
    #             ("use_sim_time", use_sim_time),
    #             ("ign_verbosity", ign_verbosity),
    #             ("log_level", log_level),
    #         ],
    #     ),
    # ]
    moveit_config = (
        MoveItConfigsBuilder("nova2_robot", package_name="nova2_moveit")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # List of nodes to be launched
    nodes = [
        # Run the example node (C++)
        Node(
            package="aiva_executor",
            executable="aiva_executor",
            output="log",
            parameters=[
                {"use_sim_time": True},
                moveit_config.robot_description_kinematics,
            ],
        ),
    ]

    return LaunchDescription(nodes) #launch_descriptions
