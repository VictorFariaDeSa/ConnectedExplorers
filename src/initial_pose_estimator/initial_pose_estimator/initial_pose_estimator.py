import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPoseEstimator(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter("namespace","")
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)

        self.target_x = self.get_parameter('x').get_parameter_value().double_value
        self.target_y = self.get_parameter('y').get_parameter_value().double_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value


        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, f'{self.namespace}/initialpose', 10)
        time.sleep(1)
        for _ in range(5):
            self.publish_pose()
            time.sleep(1)

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.target_x
        msg.pose.pose.position.y = self.target_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info("Pose inicial publicada em (0,0,0)")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
