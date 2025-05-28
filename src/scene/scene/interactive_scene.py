#!/usr/bin/env python3
# scene/interactive_scene.py  – live cubes + TF broadcaster (continuous)

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, PlanningSceneWorld, ObjectColor
)
from moveit_msgs.srv import ApplyPlanningScene
import tf2_ros
import math
import time

# ───── cube parameters ──────────────────────────────────────────
CUBE_SIZE = 0.05                     # 5 cm
CUBE_Z    = 0.025                    # centre height
CUBE_X    = (-0.30, 0.00, 0.30)       # three x-positions

CUBE_IDS  = [f"cube{i+1}" for i in range(3)]
CUBE_COL  = (
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
    ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
)

class InteractiveScene(Node):
    def __init__(self):
        super().__init__("interactive_scene")

        # parameter to match your robot base frame  ───── NEW
        self.declare_parameter("base", "base")
        self.base = self.get_parameter("base").get_parameter_value().string_value

        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # internal pose cache              ───── NEW
        self.cube_pose_cache = {}

        # 1) build & send cubes to planning scene
        self.add_cubes_to_planning_scene()

        # 2) broadcast TF at 10 Hz         ───── NEW
        self.create_timer(0.1, self.publish_all_tf)   # 10 Hz

        # 3) listen for interactive moves and update cache
        self.create_subscription(
            PlanningScene,
            "/monitored_planning_scene",
            self.scene_callback,
            10,
        )

    # ----------------------------------------------------------
    def add_cubes_to_planning_scene(self):
        prim = SolidPrimitive(type=SolidPrimitive.BOX,
                              dimensions=[CUBE_SIZE] * 3)

        cubes, colors = [], []
        for cid, x, col in zip(CUBE_IDS, CUBE_X, CUBE_COL):
            pose = Pose()
            pose.position.x = x
            pose.position.y = -0.3
            pose.position.z = CUBE_Z
            pose.orientation.w = 1.0

            co = CollisionObject()
            co.id              = cid
            co.header          = Header(frame_id=self.base)
            co.primitives      = [prim]
            co.primitive_poses = [pose]
            co.operation       = CollisionObject.ADD
            cubes.append(co)

            colors.append(ObjectColor(id=cid, color=col))

            # cache init pose               ───── NEW
            self.cube_pose_cache[cid] = pose

        scene = PlanningScene()
        scene.is_diff = True
        scene.world   = PlanningSceneWorld(collision_objects=cubes)
        scene.object_colors = colors

        cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        cli.wait_for_service()
        cli.call_async(ApplyPlanningScene.Request(scene=scene))
        self.get_logger().info("Cubes added to planning scene.")
        time.sleep(0.3)

    # ----------------------------------------------------------
    def publish_all_tf(self):
        """Broadcast cached poses for every cube (10 Hz)."""
        now = self.get_clock().now().to_msg()
        for cid, pose in self.cube_pose_cache.items():
            tf = TransformStamped()
            tf.header.stamp    = now
            tf.header.frame_id = self.base
            tf.child_frame_id  = cid
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation      = pose.orientation
            self.br.sendTransform(tf)

    # ----------------------------------------------------------
    def scene_callback(self, msg: PlanningScene):
        """Update cached pose when user drags cube in RViz."""
        for obj in msg.world.collision_objects:
            if obj.id in CUBE_IDS and obj.primitive_poses:
                self.cube_pose_cache[obj.id] = obj.primitive_poses[0]

# --------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = InteractiveScene()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
