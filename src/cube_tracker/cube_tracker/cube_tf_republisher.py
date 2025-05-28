#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# tf2 imports
import tf2_ros
import tf2_geometry_msgs  # registers do_transform_point
from geometry_msgs.msg import PointStamped

class CubeTfRepublisher(Node):
    def __init__(self):
        super().__init__('cube_tf_republisher')
        # 1) tf2 boilerplate
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 2) sub to the *raw* camera‐frame positions
        self.sub = self.create_subscription(
            String,            # same message type
            'cube_positions',  # raw camera positions
            self.cb, 10
        )
        # 3) pub back to the *same* topic name,
        #    but only *after* transforming to base_link
        self.pub = self.create_publisher(
            String,
            'cube_positions',
            10
        )

    def cb(self, msg: String):
        try:
            cubes = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
            return

        transformed = []
        for c in cubes:
            p_cam = PointStamped()
            p_cam.header.frame_id = 'camera_link'
            p_cam.header.stamp    = self.get_clock().now().to_msg()
            p_cam.point.x        = c['x']
            p_cam.point.y        = c['y']
            p_cam.point.z        = c.get('z', 0.0)

            try:
                # look up camera_link → base_link at “now”
                t = self.tf_buffer.lookup_transform(
                    'base_link',
                    p_cam.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                p_base = tf2_geometry_msgs.do_transform_point(p_cam, t)
                transformed.append({
                    'id': c['id'],
                    'x' : p_base.point.x,
                    'y' : p_base.point.y,
                    'z' : p_base.point.z
                })
            except Exception as e:
                self.get_logger().warn(f"TF failed for cube {c['id']}: {e}")

        # publish the transformed list *back* onto cube_positions
        out = String()
        out.data = json.dumps(transformed)
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CubeTfRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
