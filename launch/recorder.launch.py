from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("ur5_vla_dataset_recorder")) / "config" / "recorder.yaml"
    return LaunchDescription(
        [
            Node(
                package="ur5_vla_dataset_recorder",
                executable="dataset_recorder",
                name="dataset_recorder",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )

