import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_hw_robot_argument = DeclareLaunchArgument(
        'use_hw_robot',
        default_value='false'
    )
    use_hw_robot_condition=IfCondition(LaunchConfiguration('use_hw_robot'))
    not_use_hw_robot_condition=UnlessCondition(LaunchConfiguration('use_hw_robot'))
    
    moveit_config = (
        MoveItConfigsBuilder("nova2_with_gripper")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
        # "disable_capabilities": True,B
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": True
    }

  
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("aiva_executor") + "/config/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )
    
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "dummy_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("nova2_with_gripper_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[moveit_config.robot_description],
        condition=not_use_hw_robot_condition)
    # # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "gripper_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
                condition=not_use_hw_robot_condition
            )
        ]

    dobot_launch_file = os.path.join(
        get_package_share_directory('cr_robot_ros2'),
        'launch',
        'dobot_bringup_ros2.launch.py'
    )
    return LaunchDescription(
        [
            use_hw_robot_argument,
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dobot_launch_file),
            condition=use_hw_robot_condition
        ),
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            start_joint_state_publisher_cmd
        ]
        + load_controllers
    )
