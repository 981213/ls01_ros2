from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    lidar_params = PathJoinSubstitution(
        [
            FindPackageShare("ls01"),
            "config",
            "lsn10p.yaml",
        ]
    )

    return LaunchDescription([
        Node(
            package='ls01',
            executable="lsn10p",
            parameters=[lidar_params],
            output="screen",
        )])
