#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CubeListener(Node):
    def __init__(self):
        super().__init__('cube_listener')
        # Subscribe to JSON messages on cube_positions
        self.sub = self.create_subscription(
            String,
            'cube_positions_cam',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            cubes = json.loads(msg.data)
            for cube in cubes:
                # Include z-coordinate in log output
                z_val = cube.get('z', None)
                if z_val is not None:
                    self.get_logger().info(
                        f"Cube {cube['id']} at x={cube['x']}, y={cube['y']}, z={cube['z']}"
                    )
                else:
                    self.get_logger().info(
                        f"Cube {cube['id']} at x={cube['x']}, y={cube['y']} (no z)"
                    )
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CubeListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

