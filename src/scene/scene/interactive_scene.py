#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class InteractiveScene(Node):
    def __init__(self):
        super().__init__('interactive_scene')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.timer = self.create_timer(1.0, self.publish_object)
        self.get_logger().info('InteractiveScene node started.')

    def publish_object(self):
        obj = CollisionObject()
        obj.header = Header()
        obj.header.frame_id = "base_link"  # OBS: sjekk at dette matcher din robot!
        obj.id = "table_1"

        # Bord: 2.0 m x 1.0 m x 0.05 m
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 1.0, 0.05]

        # Midten 
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        self.publisher.publish(obj)
        self.get_logger().info('Published table collision object.')

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveScene()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()