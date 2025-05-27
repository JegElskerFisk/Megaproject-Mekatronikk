#!/usr/bin/env python3
# scene/interactive_scene.py - Kun kuber (uten bord)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
import time

BASE = "base_link"  # juster om nødvendig

# ───────── geometri ──────────────────────────────────────────────
CUBE_SIZE = 0.05  # kantlengde (m)
CUBE_Z = 0.025  # høyde (halv høyde over origo)
CUBE_X = (-0.30, 0.0, 0.30)  # posisjoner langs x-aksen

CUBE_COL = (
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # rød
    ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # grønn
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # blå
)

class InteractiveScene(Node):
    def __init__(self):
        super().__init__("interactive_scene")

        # ── 1. Kubene ──────────────────────────────────────────
        prim_cube = SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[CUBE_SIZE] * 3
        )

        cube_objs = []
        cube_colors = []

        for i, (x, col) in enumerate(zip(CUBE_X, CUBE_COL), start=1):
            cube = CollisionObject()
            cube.id = f"cube{i}"
            cube.header = Header(frame_id=BASE)

            pose = Pose()
            pose.position.x = x
            pose.position.z = CUBE_Z
            pose.orientation.w = 1.0

            cube.primitives = [prim_cube]
            cube.primitive_poses = [pose]
            cube.operation = CollisionObject.ADD
            cube_objs.append(cube)

            cube_colors.append(ObjectColor(id=cube.id, color=col))

        # ── 2. PlanningScene-diff ──────────────────────────────
        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld(collision_objects=cube_objs)
        scene.object_colors = cube_colors

        # ── 3. Send til move_group ─────────────────────────────
        cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        cli.wait_for_service()
        self.get_logger().info("Sender PlanningScene-diff med kuber...")
        cli.call_async(ApplyPlanningScene.Request(scene=scene))

        # Vent litt for at meldingen skal rekke frem
        time.sleep(0.4)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    InteractiveScene()

if __name__ == "__main__":
    main()