import numpy as np
import rclpy.time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation as R

from laser_assembler.scan_assembler import ScanAssembler


class FakeBuffer:
    """Imite tf2_ros.Buffer avec une transformation fixe."""

    def __init__(self, tf):
        self.tf = tf
        self.calls = []

    def lookup_transform(self, target, source, time, timeout=None):
        self.calls.append((target, source))
        return self.tf


def make_tf(psi, translation):
    tf = TransformStamped()
    q = R.from_euler('y', psi).as_quat()
    tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
    tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
    tf.transform.translation.x = translation[0]
    tf.transform.translation.y = translation[1]
    tf.transform.translation.z = translation[2]
    return tf


def make_scan(ranges):
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.angle_min = 0.0
    scan.angle_max = np.pi / 2
    scan.angle_increment = np.pi / 2 / (len(ranges) - 1)
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [float(r) for r in ranges]
    return scan


def test_identity_transform_keeps_scan_points():
    asm = ScanAssembler(FakeBuffer(make_tf(0.0, (0, 0, 0))))
    assert asm.add_scan(make_scan([1.0, 1.0, 1.0]))
    xyz = asm.clouds[0]
    np.testing.assert_allclose(xyz[0], [1, 0, 0], atol=1e-6)
    np.testing.assert_allclose(xyz[-1], [0, 1, 0], atol=1e-6)


def test_transform_applies_rotation_then_translation():
    # Un point lidar en (1, 0, 0), rotation de 90 deg autour de y : x -> -z
    asm = ScanAssembler(FakeBuffer(make_tf(np.pi / 2, (0.04, 0, 0.034))))
    asm.add_scan(make_scan([1.0, 1.0, 1.0]))
    np.testing.assert_allclose(asm.clouds[0][0], [0.04, 0.0, 0.034 - 1.0], atol=1e-6)


def test_lookup_uses_fixed_frame_as_target():
    buf = FakeBuffer(make_tf(0.0, (0, 0, 0)))
    ScanAssembler(buf, fixed_frame='r_robot').add_scan(make_scan([1.0, 1.0]))
    assert buf.calls == [('r_robot', 'laser')]


def test_invalid_ranges_are_removed():
    asm = ScanAssembler(FakeBuffer(make_tf(0.0, (0, 0, 0))))
    asm.add_scan(make_scan([1.0, float('inf'), 50.0, 1.0]))
    assert asm.n_points == 2


def test_pointcloud2_and_clear():
    asm = ScanAssembler(FakeBuffer(make_tf(0.0, (0, 0, 0))))
    asm.add_scan(make_scan([1.0, 1.0, 1.0]))
    asm.add_scan(make_scan([2.0, 2.0, 2.0]))
    msg = asm.to_pointcloud2()
    assert msg.header.frame_id == 'r_robot'
    assert msg.width == 6
    asm.clear()
    assert asm.n_scans == 0
    assert asm.to_pointcloud2().width == 0


def test_max_scans_is_a_circular_buffer():
    asm = ScanAssembler(FakeBuffer(make_tf(0.0, (0, 0, 0))), max_scans=2)
    for r in (1.0, 2.0, 3.0):
        asm.add_scan(make_scan([r, r]))
    assert asm.n_scans == 2
    assert asm.clouds[0][0][0] == 2.0
