#!/usr/bin/env python3
# scene/interactive_scene.py – kun CollisionObject

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
import time

BASE_FRAME = "base_link"   # juster om nødvendig

CUBE_SIZE  = 0.05          # 5 cm kube
CUBE_Z     = 0.05/ 2.0

# tre posisjoner langs x-aksen (30 cm mellomrom)
X_POS = [-0.30, 0.0, 0.30]


class InteractiveScene(Node):
    def __init__(self):
        super().__init__("interactive_scene")

        # publisher til /collision_object
        self.obj_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10)

        # kube-geometri (gjenbrukes for alle tre)
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]

        # legg inn tre kuber
        for idx, x in enumerate(X_POS, start=1):
            cube_id = f"cube{idx}"

            pose = Pose()
            pose.position.x = x
            pose.position.y = 0.5
            pose.position.z = CUBE_Z
            pose.orientation.w = 1.0

            obj = CollisionObject()
            obj.id = cube_id
            obj.header = Header(frame_id=BASE_FRAME)
            obj.primitives = [prim]
            obj.primitive_poses = [pose]
            obj.operation = CollisionObject.ADD

            self.obj_pub.publish(obj)
            self.get_logger().info(f"Publiserte {cube_id}")

        # gi ROS litt tid til å sende alle meldinger før vi avslutter
        time.sleep(0.5)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    InteractiveScene()


if __name__ == "__main__":
    main()
