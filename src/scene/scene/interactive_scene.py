#!/usr/bin/env python3
# scene/interactive_scene.py  –  kuber + TF-kringkasting

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, PlanningSceneWorld, ObjectColor
)
from moveit_msgs.srv import ApplyPlanningScene
import tf2_ros
import time

BASE = "base_link"                 # robotens base-frame

# ───── kubeparametre ──────────────────────────────────────────────
CUBE_SIZE = 0.05                   # 5 cm kant
CUBE_Z    = 0.025                  # senterhøyde
CUBE_X    = (-0.30, 0.0, 0.30)     # tre posisjoner

CUBE_IDS  = [f"cube{i+1}" for i in range(3)]
CUBE_COL  = (
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),   # rød
    ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),   # grønn
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),   # blå
)

class InteractiveScene(Node):
    def __init__(self):
        super().__init__("interactive_scene")

        # TF-broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # 1) bygg og send kubene til PlanningScene
        self.add_cubes_to_planning_scene()

        # 2) kringkast initiale TF-rammer
        self.broadcast_initial_tf()

        # 3) lytt på fremtidige diff-meldinger for å følge interaktiv flytting
        self.create_subscription(
            PlanningScene,
            "/monitored_planning_scene",
            self.scene_callback,
            10,
        )

    # ----------------------------------------------------------
    def add_cubes_to_planning_scene(self):
        prim_cube = SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[CUBE_SIZE] * 3
        )

        cube_objs   = []
        cube_colors = []

        for cid, x, col in zip(CUBE_IDS, CUBE_X, CUBE_COL):
            pose = Pose()
            pose.position.x = x
            pose.position.z = CUBE_Z
            pose.orientation.w = 1.0

            cube = CollisionObject()
            cube.id              = cid
            cube.header          = Header(frame_id=BASE)
            cube.primitives      = [prim_cube]
            cube.primitive_poses = [pose]
            cube.operation       = CollisionObject.ADD
            cube_objs.append(cube)

            cube_colors.append(ObjectColor(id=cid, color=col))

        scene = PlanningScene()
        scene.is_diff        = True
        scene.world          = PlanningSceneWorld(collision_objects=cube_objs)
        scene.object_colors  = cube_colors

        cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        cli.wait_for_service()
        cli.call_async(ApplyPlanningScene.Request(scene=scene))
        self.get_logger().info("Kuber lagt til i planning-scenen.")
        time.sleep(0.3)   # la meldingen gå igjennom

    # ----------------------------------------------------------
    def broadcast_initial_tf(self):
        """Send én TF-melding for hver kube (init-posisjon)."""
        for cid, x in zip(CUBE_IDS, CUBE_X):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = BASE
            tf.child_frame_id  = cid
            tf.transform.translation.x = x
            tf.transform.translation.z = CUBE_Z
            tf.transform.rotation.w    = 1.0
            self.br.sendTransform(tf)
        self.get_logger().info("Initiale TF-rammer sendt.")

    # ----------------------------------------------------------
    def scene_callback(self, msg: PlanningScene):
        """Oppdater TF når kuber flyttes interaktivt i RViz."""
        for obj in msg.world.collision_objects:
            if obj.id in CUBE_IDS and obj.primitive_poses:
                pose = obj.primitive_poses[0]
                tf = TransformStamped()
                tf.header.stamp = msg.scene_time
                tf.header.frame_id = BASE
                tf.child_frame_id  = obj.id
                tf.transform.translation = pose.position
                tf.transform.rotation   = pose.orientation
                self.br.sendTransform(tf)

# --------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = InteractiveScene()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
