#!/usr/bin/env python3
"""
interactive_scene.py
Publiserer:

• én statisk TF-ramme «table»
• tre interaktive kuber («cube1»-«cube3») som kan dras i RViz
  – hver kube publiseres som egen TF-ramme

Bruk:
$ ros2 run scene interactive_scene
Åpne RViz → legg til «TF» + «InteractiveMarkers».
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
)


class InteractiveScene(Node):
    def __init__(self):
        super().__init__("interactive_scene")

        # --- TF-broadcasters ------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # --- Interactive Marker-server -------------------------------------
        self.server = InteractiveMarkerServer(self, "interactive_cubes")

        # Publiser bordet
        self._publish_table_tf()

        # Lag tre kuber
        for idx in range(1, 4):
            self._make_cube(f"cube{idx}")

        self.get_logger().info(
            "Scene klar – dra kubene i RViz (TF-rammene heter cube1-cube3)"
        )

    # ------------------------------------------------------------------ #
    #  Bord                                                              #
    # ------------------------------------------------------------------ #
    def _publish_table_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "table"
        t.transform.translation.z = 0.3  # 30 cm over world-origo
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #
    #  Kube                                                              #
    # ------------------------------------------------------------------ #
    def _make_cube(self, name: str):
        im = InteractiveMarker()
        im.header.frame_id = "world"
        im.name = name
        im.description = name
        im.scale = 0.2  # RViz-skala på markøren (ikke kube­størrelsen)

        # Startposisjon midt foran robot
        im.pose.position.x = 0.5
        im.pose.position.y = 0.1 * (int(name[-1]) - 2)  # −0.1, 0, +0.1 m
        im.pose.orientation.w = 1.0

        # Dra i XY-planet
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.w = 1.0
        ctrl.orientation.y = 1.0        # definerer planet
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        ctrl.always_visible = True

        # Selve kube-visualen
        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = cube.scale.y = cube.scale.z = 0.05  # 5 cm
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.0, 0.7, 0.7, 1.0
        ctrl.markers.append(cube)
        im.controls.append(ctrl)

        self.server.insert(im, self._cube_feedback)
        self.server.applyChanges()

    def _cube_feedback(self, feedback):
        """Når brukeren slutter å dra en kube → publiser ny TF."""
        if feedback.event_type != feedback.MOUSE_UP:
            return

        pose = feedback.pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = feedback.marker_name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = InteractiveScene()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
