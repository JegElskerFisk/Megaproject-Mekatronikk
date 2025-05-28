# File: ur_bringup.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hw = LaunchConfiguration("use_fake_hardware")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    declare_args = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            description="UR-type, e.g., ur3, ur5e, etc."
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="IP address of the robot. Ignored in fake mode."
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for simulation."
        ),

        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="The controller to start at launch"
        ),
    ]

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("ur_robot_driver").find("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_fake_hw,
            "launch_rviz": "false"
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("ur_moveit_config").find("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "true"
        }.items(),
    )

    return LaunchDescription(declare_args + [ur_driver, moveit])
