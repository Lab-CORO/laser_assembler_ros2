import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS Messages
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from encodeur.srv import AssembleCloud

# TF in ROS 2
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np

# Open3D for point cloud accumulation
import open3d as o3d

# For converting LaserScan -> point array
import math
import threading

from .laser_scan_to_array import laserscan_to_array

try:
    import ros2_numpy
except ImportError:
    raise ImportError("ros2_numpy is required for point cloud conversions. "
                      "Install via `pip install ros2_numpy` in your ROS environment.")

def laserscan_to_xyz(laserscan_msg):
    """
    Convert LaserScan to an Nx3 NumPy array of [x, y, z=0].
    Invalid ranges (inf or out of [range_min, range_max]) are discarded.
    """
    scan_np = laserscan_to_array(laserscan_msg)
    return scan_np 

class LaserAssemblerNode(Node):
    def __init__(self):
        super().__init__('laser_assembler')

        # List (or buffer) of partial clouds (Open3D) that we accumulate
        self.clouds_buffer = []
        self.buffer_lock = threading.Lock()

        # TF Buffer + Listener for transform lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to LaserScan
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/perception/transformed_scan',          # or your laser topic name
            self.scan_callback,
            qos_profile
        )

        # Create a service server for assembling all clouds
        self.assemble_srv = self.create_service(
            AssembleCloud,
            'assemble_cloud',
            self.assemble_cloud_callback
        )

        # Optional: Which frame to transform everything into
        self.global_frame = 'r_robot'

        self.get_logger().info('LaserAssemblerNode initialized.')


    def scan_callback(self, scan_msg: LaserScan):
        """
        Callback for each incoming LaserScan.
        1) Find the transform from scan_msg.header.frame_id -> self.global_frame
        2) Convert LaserScan to a local point cloud
        3) Transform to the global frame
        4) Store the new cloud in self.clouds_buffer
        """
        # Convert LaserScan -> Nx3 array
        local_points = laserscan_to_xyz(scan_msg)
        if local_points.size == 0:
            self.get_logger().warn('Received LaserScan with 0 valid points.')
            return

        # Attempt to get the transform from the scan frame to the global frame
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                scan_msg.header.frame_id,
                self.global_frame,
                rclpy.time.Time()        # or scan_msg.header.stamp if TF is in sync
            )
        except tf2_ros.LookupException as e:
            self.get_logger().error(f'TF LookupException: {str(e)}')
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f'TF ConnectivityException: {str(e)}')
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'TF ExtrapolationException: {str(e)}')
            return

        # Convert transform to a 4x4 matrix
        # T = transform_stamped_to_matrix(transform_stamped)
        T = ros2_numpy.numpify(transform_stamped.transform)
        # Convert local point array -> Open3D point cloud
        local_pcd = o3d.geometry.PointCloud()
        local_pcd.points = o3d.utility.Vector3dVector(local_points)

        # Transform to global frame
        local_pcd.transform(T)

        # TEST rotation 90 selon Y
        R = local_pcd.get_rotation_matrix_from_xyz((0, 0, np.pi))
        local_pcd.rotate(R, center=(0, 0, 0))

        # Store in our list
        # with self.buffer_lock:
        self.clouds_buffer.append(local_pcd)
        print("scan load")

    def assemble_cloud_callback(self, request, response):
        """
        This service fuses ALL accumulated partial clouds in 'clouds_buffer' 
        and returns the fused result as sensor_msgs/PointCloud2.
        """
        # Grab all partial clouds in a thread-safe manner
        with self.buffer_lock:
            partial_clouds = self.clouds_buffer
            self.clouds_buffer = []  # clear after assembly, or keep them if you prefer

        if len(partial_clouds) == 0:
            self.get_logger().warn('No partial clouds to assemble. Returning empty cloud.')
            # Return an empty PointCloud2
            empty_cloud = ros2_numpy.msgify(PointCloud2, np.zeros((0, 3), dtype=np.float32))
            empty_cloud.frame_id=self.global_frame
        
            response.assembled_cloud = empty_cloud
            return response

        partial_clouds_np = [np.asarray(pc.points) for pc in partial_clouds]

        xyz_array = np.vstack(partial_clouds_np)
      
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.height = 1
        msg.width = xyz_array.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12
        msg.row_step = msg.point_step * xyz_array.shape[0]

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = xyz_array.astype(np.float32).tobytes()
        
        # print(xyz_array)
        # fused_msg = ros2_numpy.msgify(PointCloud2, xyz_array)
        # fused_msg.frame_id=self.global_frame
        
        response.assembled_cloud = msg

        self.get_logger().info(f'Assembled {len(partial_clouds)} clouds into {len(xyz_array)} points.')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LaserAssemblerNode()
    rclpy.spin(node)
    rclpy.shutdown()
