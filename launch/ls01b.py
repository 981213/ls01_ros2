from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    lidar_params = PathJoinSubstitution(
        [
            FindPackageShare("ls01"),
            "config",
            "ls01b.yaml",
        ]
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name='lslidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ls01',
                    plugin='LS01::LS01B',
                    name='ls01b',
                    parameters=[lidar_params]),
            ],
            output='screen',
        )
    ])
