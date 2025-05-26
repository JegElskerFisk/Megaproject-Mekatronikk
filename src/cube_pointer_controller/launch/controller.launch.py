from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_share = get_package_share_directory("cube_pointer_controller")
    param_file = Path(pkg_share) / "config" / "params.yaml"

    demo_move = Node(
        package="cube_pointer_controller",
        executable="controller_node",
        name="cube_pointer_controller",
        output="screen",
        parameters=[param_file] if param_file.exists() else [],
    )

    return LaunchDescription([demo_move])
