# static_camera_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) publish camera_link as a fixed child of the moving wrist link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            output='screen',
            arguments=[
                '0.07','0.0','0.07',# X Y Z (meters)
                '0','0','0.0',     # roll pitch yaw (radians)
                'tool0',             # parent = your wrist link
                'camera_link'        # child  = the camera frame you use
            ]
        ),

        # 2) start your cube_listener (or cube_detector) node
        Node(
            package='cube_tracker',
            executable='cube_listener',
            name='cube_listener',
            output='screen'
        ),

        Node(
            package='cube_tracker',
            executable='cube_tf_republisher',
            name='cube_tf_republisher',
            output='screen'
        ),

        Node(
            package='cube_tracker',
            executable='cube_detector',
            name='cube_detector',
            output='screen'
        ),
    ])

