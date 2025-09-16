import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from laser_geometry import LaserProjection
import tf2_ros
from tf2_ros import TransformException, TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
import threading
from scipy.spatial import cKDTree
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion

class PauseAndCapture(Node):
    def __init__(self):
        super().__init__('pause_and_capture')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.laser_projector = LaserProjection()

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT)

        # Set up the subscription for LaserScan message
        # HINT: Subscribe on the '/scan' topic
        self.subscription = ...

        # Create a publisher for PointCloud2 messages
        # HINT: Publish on the '/accumulated_cloud' topic
        self.pc_pub = ...

        # Create a publisher for ICP merged cloud
        # HINT: Publish on the '/icp_merged_cloud' topic
        self.icp_pub = ...

        self.accumulated_points = []
        self.icp_accumulated_points = []

        self.capture_enabled = False
        self.latest_scan = None
        self.delay_timer = None

        self.input_thread = threading.Thread(target=self.key_press_listener, daemon=True)
        self.input_thread.start()

        self.get_logger().info("PauseAndCapture node started. Press Enter to capture a scan.")

    def key_press_listener(self):
        while True:
            input(">> Press Enter to capture scan: ")
            self.capture_enabled = True

    def scan_callback(self, scan_msg: LaserScan):
        if not self.capture_enabled:
            return

        self.latest_scan = scan_msg
        self.capture_enabled = False

        if self.delay_timer:
            self.delay_timer.cancel()
        self.delay_timer = self.create_timer(0.1, self.delayed_transform_lookup)

    def delayed_transform_lookup(self):
        self.delay_timer.cancel()
        scan_msg = self.latest_scan
        self.latest_scan = None

        try:
            cloud_in_laser = self.laser_projector.projectLaser(scan_msg)

            #TODO:
            # Perform a lookup to transform the point cloud from its original frame to the 'odom' frame
            transform = self.tf_buffer.lookup_transform(
                ..., # Target frame (where do you want to transform to?)
                ...,# Source frame (the point cloud's original frame)
                ...,  # Timestamp of the scan message to ensure proper time synchronization
                ...  # Timeout of 0.5 seconds to wait for the transform
            )

            transformed_points = ... # Transform the point cloud with the transform

            if self.icp_accumulated_points:
                icp_aligned = self.perform_icp(transformed_points, self.icp_accumulated_points)
                self.icp_accumulated_points.extend(icp_aligned)
                self.publish_icp_merged_cloud(scan_msg.header.stamp)
                self.get_logger().info(f"ICP-aligned and merged {len(icp_aligned)} points.")
            else:
                self.icp_accumulated_points.extend(transformed_points)
                self.publish_icp_merged_cloud(scan_msg.header.stamp)
                self.get_logger().info(f"Initialized ICP merged cloud with {len(transformed_points)} points.")

            self.accumulated_points.extend(transformed_points)
            self.publish_accumulated_cloud(scan_msg.header.stamp)
            self.get_logger().info(f"Captured and transformed {len(transformed_points)} points.")

        except TransformException as ex:
            self.get_logger().warn(f"Transform failed after delay: {str(ex)}")




    #TODO: Complete the rotate_point_euler in transform_pointcloud2 functios
    #Note that ros iherently processes point clouds in 3d even though the robot's point cloud is in 2d.

    def transform_pointcloud2(self, cloud_msg: PointCloud2, transform: TransformStamped) -> list[tuple[int, int, int]]:
        """Transform a point cloud using Euler angles from a given quaternion."""

        def rotate_point_euler(x, y, z, roll, pitch, yaw) -> tuple[int, int, int]:
            """Rotate a point (x, y, z) using Euler angles (roll, pitch, yaw)."""
            #TODO:
            #using the roll,pitch and yaw construct the Rx , Ry, Rz matrix

            #TODO:
            # Combined rotation matrix (left yaw, middle pitch, rightmost roll)

            #TODO:
            # Apply the rotation to the point

            return ...


        # Extract translation and rotation (quaternion) from the transform method
        ...

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        # Hint: Use the euler_from_quaternion
        ...

        # Transform the point cloud using Euler rotation
        transformed_points = []
        for pt in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            # Get values of pt
            ...            
            
            #TODO:
            # Apply rotation to the point using Euler angles use the rotate point euler function
            
            # Append transformed point
            transformed_points.append((...))

        return transformed_points

    def perform_icp(self, previous_points, current_points, max_iterations=20, tolerance=1e-4):
        src = np.array(previous_points)
        dst = np.array(current_points)

        ERROR = []
        prev_error = float('inf')
        prev_error = float(10000.0)
        counter_ = 0
        
        # Write the ICP loop here
        # Hint: cKdtree() function will be useful to get distances and indices
        ...

        return src.tolist()
    

    #TODO: Same function from HW 4 :)
    # Curr = Source, Prev = Target 
    def svd_estimation(self, previous_points, current_points):
        pass

        


    def publish_accumulated_cloud(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, self.accumulated_points)
        self.pc_pub.publish(cloud_msg)
        self.get_logger().info("Published accumulated cloud.")

    def publish_icp_merged_cloud(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = "odom"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, self.icp_accumulated_points)
        self.icp_pub.publish(cloud_msg)
        self.get_logger().info("Published ICP merged cloud.")

def main(args=None):
    rclpy.init(args=args)
    node = PauseAndCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
