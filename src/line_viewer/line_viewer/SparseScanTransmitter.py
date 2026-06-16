import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from mobile_robot_interfaces.msg import SparseMappingData


class SparseScanTransmitter(Node):
    def __init__(self):
        super().__init__('sparse_scan_transmitter')

        # Get robot namespace to identify who is sending the data
        self.robot_name = self.get_namespace().strip('/')
        if not self.robot_name:
            self.robot_name = 'unknown_robot'

        self.distance_threshold = 0.25  # Transmit every 2 meters
        self.angle_threshold = 0.785   # Transmit every ~45 degrees

        self.max_translation_variance = 0.1  # V_T_max (meters squared)
        self.max_rotation_variance = 0.05    # V_R_max (radians squared)

        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.latest_scan = None

        # Local Subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)

        self.net_pub = self.create_publisher(SparseMappingData, '/network_mapping_data', 10)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback(self, msg:PoseWithCovarianceStamped):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_yaw = self.get_yaw(msg.pose.pose.orientation)

        if self.last_x is None:
            self.last_x, self.last_y, self.last_yaw = current_x, current_y, current_yaw
            return

        delta_d = math.sqrt((current_x - self.last_x)**2 + (current_y - self.last_y)**2)
        delta_yaw = abs(current_yaw - self.last_yaw)

        if delta_d >= self.distance_threshold or delta_yaw >= self.angle_threshold:
            if self.latest_scan is not None:
                cov = msg.pose.covariance
                var_x = cov[0]   # x variance
                var_y = cov[7]   # y variance
                var_yaw = cov[35] # yaw variance

                # Decoupled Minimum Confidence Logic
                c_pos = max(0.0, 1.0 - ((var_x + var_y) / self.max_translation_variance))
                c_rot = max(0.0, 1.0 - (var_yaw / self.max_rotation_variance))
                alpha_value = float(min(c_pos, c_rot))

                unified_msg = SparseMappingData()

                # Master Header
                unified_msg.header.stamp = self.get_clock().now().to_msg()
                unified_msg.header.frame_id = self.robot_name

                # Attach Data
                unified_msg.pose = msg.pose.pose
                unified_msg.scan = self.latest_scan
                unified_msg.alpha = alpha_value

                # Note: We override the scan's internal frame_id to match the robot
                # so the supervisor knows whose laser frame this belongs to.
                unified_msg.scan.header.frame_id = f"{self.robot_name}/base_scan"

                # --- Transmit once ---
                self.net_pub.publish(unified_msg)

                self.get_logger().info(f'Transmitted unified mapping payload. Alpha: {alpha_value:.2f}')

                # Reset thresholds
                self.last_x, self.last_y, self.last_yaw = current_x, current_y, current_yaw

def main(args=None):
    rclpy.init(args=args)
    node = SparseScanTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
