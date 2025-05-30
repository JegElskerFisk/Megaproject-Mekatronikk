#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
from ament_index_python.packages import get_package_share_directory

class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        # JSON-string publisher
        self.pub = self.create_publisher(String, 'cube_positions_cam', 10)
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(1,cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            exit(1)

                # Load calibration file
        resource_path = os.path.join(
            os.path.dirname(__file__), '..', 'resource', 'camera_calib.npz'
        )
        self.load_calibration(resource_path)
        self.compute_homography()

        # Real-world cube height (cm)
        self.cube_size_cm = 3.0

        # Timer for looped capture
        self.timer = self.create_timer(0.1, self.capture_loop)

    def load_calibration(self, path):
        try:
            data = np.load(path)
            # Handle different key names
            if 'camera_matrix' in data:
                self.camera_matrix = data['camera_matrix']
            elif 'cameraMatrix' in data:
                self.camera_matrix = data['cameraMatrix']
            else:
                raise KeyError('camera_matrix / cameraMatrix not found')

            if 'dist_coeffs' in data:
                self.dist_coeffs = data['dist_coeffs']
            elif 'dist' in data:
                self.dist_coeffs = data['dist']
            else:
                raise KeyError('dist_coeffs / dist not found')

            self.calib_loaded = True
            self.get_logger().info(f'Loaded calibration from {path}')
        except Exception as e:
            self.calib_loaded = False
            self.get_logger().warn(f'Calibration load failed ({path}): {e}')

    def compute_homography(self):
        img_pts = np.array([[100,100],[500,100],[500,500],[100,500]], dtype=np.float32)
        world_pts = np.array([
    [ 0 - 20,  0 - 20],   # = [-20, -20]
    [40 - 20,  0 - 20],   # = [ 20, -20]
    [40 - 20, 40 - 20],   # = [ 20,  20]
    [ 0 - 20, 40 - 20],   # = [-20,  20]
], dtype=np.float32)
        self.homography, _ = cv2.findHomography(img_pts, world_pts)
        self.homography_loaded = self.homography is not None
        if self.homography_loaded:
            self.get_logger().info("Homography computed.")

    def capture_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame capture failed")
            return

        # Undistort if calibration loaded
        if self.calib_loaded:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

        # Color segmentation
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = {
            'red':   cv2.inRange(hsv, (0,100,100),(10,255,255)) |
                     cv2.inRange(hsv,(160,100,100),(179,255,255)),
            'green': cv2.inRange(hsv,(40,50,50),(90,255,255)),
            'blue':  cv2.inRange(hsv,(100,150,0),(140,255,255))
        }

        detections = []
        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            idx = 1
            for c in contours:
                if cv2.contourArea(c) < 500 or idx > 1:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                center_px = np.array([[[x + w/2, y + h/2]]], dtype=np.float32)

                # XY mapping
                if self.homography_loaded:
                    world = cv2.perspectiveTransform(center_px, self.homography)[0][0]
                else:
                    world = center_px[0][0]

                # Z estimation (pin-hole model)
                f_px = self.camera_matrix[1,1] if self.calib_loaded else 800.0
                z_cm = (f_px * self.cube_size_cm) / float(h)

                x_m = float(world[0]) /100.0
                y_m = float(world[1]) /100.0
                z_m = float(z_cm)     / 100.0

                detections.append({
                    'id': f"{color}cube{idx}",
                    'x': x_m,
                    'y': y_m,
                    'z': z_cm
                })

                # Draw box & label
                cv2.rectangle(frame, (x,y),(x+w,y+h),(255,255,255),2)
                cv2.putText(frame, f"{color}{idx}", (x, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                idx += 1

        # Publish JSON
        msg = String()
        msg.data = json.dumps(detections)
        self.pub.publish(msg)

    def test_publish(self):
        sample = [{ 'id':'test', 'x':0.0,'y':0.0,'z':0.0 }]
        msg = String()
        msg.data = json.dumps(sample)
        self.get_logger().info("Publishing test data")
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
