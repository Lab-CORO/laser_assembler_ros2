"""Assemblage de balayages LaserScan 2D en un nuage de points 3D (sans service)."""

import numpy as np
import rclpy.duration
import rclpy.time
import tf2_ros
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from .laser_scan_to_array import laserscan_to_array


def transform_to_matrix(transform):
    """Convertit un geometry_msgs/Transform en matrice homogene 4x4."""
    q = transform.rotation
    t = transform.translation
    T = np.eye(4)
    T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class ScanAssembler:
    """Accumule des LaserScan exprimes dans `fixed_frame`, puis les fusionne.

    Pour chaque scan, la transformation ^fixed_T_scan est lue dans le buffer TF,
    comme le faisait le node laser_assembler, mais ici sans service ROS.
    """

    def __init__(self, tf_buffer, fixed_frame='r_robot', max_scans=None):
        self.tf_buffer = tf_buffer
        self.fixed_frame = fixed_frame
        self.max_scans = max_scans
        self.clouds = []   # liste de tableaux (N, 3) dans fixed_frame

    @property
    def n_scans(self):
        return len(self.clouds)

    @property
    def n_points(self):
        return sum(len(c) for c in self.clouds)

    def _lookup(self, source_frame, stamp):
        """Transformation fixed_frame <- source_frame au temps du scan.

        Si TF n'a pas encore de donnee a cet instant, on utilise la plus recente.
        """
        timeout = rclpy.duration.Duration(seconds=1.0)
        try:
            return self.tf_buffer.lookup_transform(
                self.fixed_frame, source_frame, rclpy.time.Time.from_msg(stamp),
                timeout=timeout)
        except tf2_ros.ExtrapolationException:
            return self.tf_buffer.lookup_transform(
                self.fixed_frame, source_frame, rclpy.time.Time())

    def add_scan(self, scan):
        """Ajoute un LaserScan. Retourne False si le scan n'a pu etre transforme."""
        pts_L = laserscan_to_array(scan, remove_invalid_ranges=True)
        if pts_L.size == 0:
            return False

        try:
            tf = self._lookup(scan.header.frame_id, scan.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False

        T = transform_to_matrix(tf.transform)
        pts_R = pts_L @ T[:3, :3].T + T[:3, 3]

        self.clouds.append(pts_R.astype(np.float32))
        if self.max_scans is not None and len(self.clouds) > self.max_scans:
            self.clouds.pop(0)   # tampon circulaire : on oublie le plus ancien
        return True

    def to_pointcloud2(self, stamp=None):
        """Fusionne tous les scans en un PointCloud2 dans fixed_frame."""
        xyz = (np.vstack(self.clouds) if self.clouds
               else np.zeros((0, 3), dtype=np.float32))

        msg = PointCloud2()
        msg.header = Header(frame_id=self.fixed_frame)
        if stamp is not None:
            msg.header.stamp = stamp
        msg.height = 1
        msg.width = xyz.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * xyz.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = xyz.astype(np.float32).tobytes()
        return msg

    def clear(self):
        self.clouds.clear()
