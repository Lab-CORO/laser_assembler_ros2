"""Node facultatif : enveloppe ROS (topic + service) autour de ScanAssembler.

Le laboratoire n'en a plus besoin : lidar_traitement utilise directement la
classe ScanAssembler. Ce node reste disponible pour assembler des scans depuis
la ligne de commande (ros2 service call /assemble_cloud ...).
"""

import threading

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

from encodeur.srv import AssembleCloud

from .scan_assembler import ScanAssembler


class LaserAssemblerNode(Node):
    def __init__(self):
        super().__init__('laser_assembler')

        self.buffer_lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer()
        # spin_thread=True : add_scan peut attendre la TF sans bloquer sa reception
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.assembler = ScanAssembler(self.tf_buffer, fixed_frame='r_robot')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/perception/transformed_scan', self.scan_callback, qos_profile)
        self.assemble_srv = self.create_service(
            AssembleCloud, 'assemble_cloud', self.assemble_cloud_callback)

        self.get_logger().info('LaserAssemblerNode initialized.')

    def scan_callback(self, scan_msg):
        with self.buffer_lock:
            if not self.assembler.add_scan(scan_msg):
                self.get_logger().warn('Scan ignore (aucun point valide ou TF indisponible).')

    def assemble_cloud_callback(self, request, response):
        with self.buffer_lock:
            response.assembled_cloud = self.assembler.to_pointcloud2(
                self.get_clock().now().to_msg())
            self.get_logger().info(
                f'Assembled {self.assembler.n_scans} scans into '
                f'{self.assembler.n_points} points.')
            self.assembler.clear()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LaserAssemblerNode()
    rclpy.spin(node)
    rclpy.shutdown()
