#!/usr/bin/env python3
# table_publisher.py - Publiserer et statisk bord til MoveIt-planleggingsscenen

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, ColorRGBA
from moveit_msgs.msg import ObjectColor
import time

class TablePublisher(Node):
    def __init__(self):
        super().__init__("table_publisher")
        
        # Opprett klient for ApplyPlanningScene-tjenesten
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, 
            "/apply_planning_scene"
        )
        
        # Vent til tjenesten er tilgjengelig
        while not self.apply_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Tjeneste ikke tilgjengelig, venter...')
        
        self.publish_table()

    def publish_table(self):
        """Bygg og publiser bordet til planleggingsscenen."""
        # ─── Konfigurer bordet ───────────────────────────────────
        table_obj = CollisionObject()
        table_obj.header = Header(frame_id="base_link")  
        table_obj.id = "table_1"
        
        # Definer geometri
        table_prim = SolidPrimitive()
        table_prim.type = SolidPrimitive.BOX
        table_prim.dimensions = [2.0, 1.0, 0.05]  # L × B × H i meter
        
        # Definer posisjon og orientering
        table_pose = Pose()
        table_pose.position.x = 0.0
        table_pose.position.y = 0.30
        table_pose.position.z = -0.025  
        table_pose.orientation.w = 1.0  
        
        table_obj.primitives = [table_prim]
        table_obj.primitive_poses = [table_pose]
        table_obj.operation = CollisionObject.ADD
        
        # ─── Opprett scenemelding ────────────────────────────────
        scene = PlanningScene()
        scene.is_diff = True
        scene.world = PlanningSceneWorld(collision_objects=[table_obj])
        
        # Legg til farge for bedre visualisering
        table_color = ObjectColor()
        table_color.id = table_obj.id
        table_color.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)  # Lys grå
        scene.object_colors = [table_color]
        
        # ─── Send til planleggingsscenen ─────────────────────────
        request = ApplyPlanningScene.Request(scene=scene)
        future = self.apply_scene_client.call_async(request)
        
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info("Bordet ble lagt til i planleggingsscenen")
        else:
            self.get_logger().error("Kunne ikke legge til bordet")
        
        
        time.sleep(0.5)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TablePublisher()
    rclpy.spin(node)  # Vil avslutte etter publish_table()

if __name__ == "__main__":
    main()