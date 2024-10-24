import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='aiva_ros_bridge',
            executable='aiva_ros_bridge',
            name='aiva_ros_bridge'),
  ])