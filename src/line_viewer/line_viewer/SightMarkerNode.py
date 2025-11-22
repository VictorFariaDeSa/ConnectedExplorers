import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from .RobotClass import RobotClass
from geometry_msgs.msg import Point
from functools import partial
from typing import Dict
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import numpy as np

class SightMarkerNode(Node):
    def __init__(self):
        super().__init__('gazebo_line_publisher')
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        
        self.declare_parameter("reference_frame", "map")
        self.reference_frame = self.get_parameter("reference_frame").value

        self.declare_parameter("line_alpha", 1.0)
        self.line_alpha = self.get_parameter("line_alpha").value

        self.declare_parameter("line_scale", 0.05)
        self.line_scale = self.get_parameter("line_scale").value

        self.declare_parameter("publisher_node_name", "visualization_marker")
        self.publisher_node_name = self.get_parameter("publisher_node_name").value

        self.declare_parameter("robots_list", [''], robot_list_descriptor)
        self.robots_list = self.get_parameter("robots_list").value
        self.robots_list = [] if self.robots_list == [''] else self.robots_list
        self.laplacian_matrix = None
        self.subscriptions_dict = {}
        self.robots_instances:Dict[str, RobotClass] = {}

        qos = QoSProfile(depth=10)

        for i, robot_name in enumerate(self.robots_list):
            topic_name = f"{robot_name}/position"             
            self.robots_instances[robot_name] = RobotClass(robot_name)
            callback_function = partial(self.listener_callback, robot_index=i)
            self.subscriptions_dict[robot_name] = self.create_subscription(
                Point,
                topic_name,
                callback_function,
                qos
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')

        self.laplacian_subscriber = self.create_subscription(
                Float64MultiArray,
                "laplacian_matrix",
                self.laplacian_matrix_cb,
                qos
            )

        self.marker_publisher = self.create_publisher(
                Marker, 
                self.publisher_node_name, 
                qos
            )
        self.timer = self.create_timer(0.1, self.publish_markers)

    def convert_ros_msg_to_numpy(self,msg: Float64MultiArray) -> np.ndarray:
        if not msg.data:
            return np.array([])
        data = np.array(msg.data)
        if msg.layout.dim:
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
            if rows * cols == len(data):
                return data.reshape((rows, cols))
        side_length = int(np.sqrt(len(data)))
        
        if side_length * side_length == len(data):
            return data.reshape((side_length, side_length))
        return data

    def laplacian_matrix_cb(self,msg):
        self.laplacian_matrix = self.convert_ros_msg_to_numpy(msg)

    def listener_callback(self, msg, robot_index):
        robot_name = self.robots_list[robot_index] 
        self.robots_instances[robot_name].Set_position(msg)



    def publish_markers(self):
        if self.laplacian_matrix is None:
            self.get_logger().info("Laplacian matrix in null")
            return
            
        for i, r_name_1 in enumerate(self.robots_list):
            for j, r_name_2 in enumerate(self.robots_list):
                if i >= j:continue
                r1 = self.robots_instances[r_name_1]
                r2 = self.robots_instances[r_name_2]
                score = self.laplacian_matrix[i, j]
                color = self.get_marker_color(score)
                marker = self.create_marker_between_robots(r1,r2,color)
                self.marker_publisher.publish(marker)


    def get_marker_color(self,score):
        score = abs(score)
        if score == 0:
            rgb_color = (1.0, 0.0, 0.0)
        else:         
            r_val = 1.0 - score
            g_val = 1.0 
            b_val = 0.0
            rgb_color = (r_val, g_val, b_val)
        return rgb_color

    def create_marker_between_robots(self,r1:RobotClass, r2:RobotClass, rgb_color):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sight_marker"
        marker.id = hash(frozenset([r1.name, r2.name])) % 1000
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.color.r, marker.color.g, marker.color.b = rgb_color
        marker.color.a = float(self.line_alpha)
        marker.scale.x = float(self.line_scale)
        
        marker.points = [r1.position, r2.position]
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = SightMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()