#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time


class InteractiveScene(Node):
    def __init__(self):
        super().__init__("interactive_scene")

        pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        # ── bygg bordet ─────────────────────────────────────────
        obj = CollisionObject()
        obj.header = Header(frame_id="base_link")    # tilpass om nødvendig
        obj.id = "table_1"

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [2.0, 1.0, 0.05]           # L × B × H i meter

        pose = Pose()
        pose.position.x = 0.0                       
        pose.position.y = 0.30
        pose.position.z = -0.03                     # senterhøyde
        pose.orientation.w = 1.0                     # ingen rotasjon

        obj.primitives      = [prim]
        obj.primitive_poses = [pose]
        obj.operation       = CollisionObject.ADD

        # ── publiser én gang ───────────────────────────────────
        pub.publish(obj)
        self.get_logger().info("Publiserte bord-objektet")

        time.sleep(0.3)  # la meldingen nå ut
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    InteractiveScene()


if __name__ == "__main__":
    main()