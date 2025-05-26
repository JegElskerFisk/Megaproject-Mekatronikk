import os
import numpy as np

import rclpy
from rclpy.node import Node

class NPZLoaderNode(Node):
    def __init__(self):
        super().__init__('npz_loader_node')
        # Parameter for sti til npz-fil
        self.declare_parameter(
            'file_path',
            os.path.join(
                os.path.dirname(__file__),
                '..', 'calibration_data.npz'
            )
        )
        file_path = self.get_parameter('file_path').get_parameter_value().string_value

        if not os.path.isfile(file_path):
            self.get_logger().error(f"Fil ikke funnet: {file_path}")
            return

        data = np.load(file_path)
        self.get_logger().info(f"Fant keys: {data.files}")

        for key in data.files:
            mat = data[key]
            # Formaterer matplotlib-lignende
            self.get_logger().info(f"{key} (shape {mat.shape}):\n{mat}")

def main(args=None):
    rclpy.init(args=args)
    node = NPZLoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#  
