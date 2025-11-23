import rclpy
from rclpy.node import Node
from functools import partial
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile
from typing import Dict
from std_msgs.msg import Float64MultiArray

from .RobotClass import RobotClass
from .MapHandler import MapHandler
from nav_msgs.msg import OccupancyGrid
from .MathHandler import MathHandler
from .Ros2Utils import numpy_matrix_to_float64multArray
from geometry_msgs.msg import Pose


class RobotsMathNode(Node):
    def __init__(self):
        super().__init__('gazebo_line_publisher')
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        
        self.declare_parameter("robots_list", [''], robot_list_descriptor)
        self.robots_list = self.get_parameter("robots_list").value
        self.robots_list = [] if self.robots_list == [''] else self.robots_list

        self.declare_parameter("max_robots_dist", 10.0)
        self.max_robots_dist = self.get_parameter("max_robots_dist").value

        self.declare_parameter("min_dist_to_wall", 0.1)
        self.min_dist_to_wall = self.get_parameter("min_dist_to_wall").value

        self.declare_parameter("max_dist_to_wall", 1.0)
        self.max_dist_to_wall = self.get_parameter("max_dist_to_wall").value

        self.declare_parameter("laplacian_topic_name", "laplacian_matrix")
        self.laplacian_topic_name = self.get_parameter("laplacian_topic_name").value

        self.declare_parameter("lambda_gradient_topic_name", "lambda2_gradient")
        self.lambda_gradient_topic_name = self.get_parameter("lambda_gradient_topic_name").value

        self.subscriptions_dict = {}
        self.robots_instances:Dict[str, RobotClass] = {}


        self.math_handler:MathHandler = None
        self.map_handler:MapHandler = None


        qos = QoSProfile(depth=10)

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.on_map_cb,
            QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        )

        for i, robot_name in enumerate(self.robots_list):
            topic_name = f"{robot_name}/position"             
            self.robots_instances[robot_name] = RobotClass(robot_name)
            callback_function = partial(self.listener_callback, robot_index=i)
            self.subscriptions_dict[robot_name] = self.create_subscription(
                Pose,
                topic_name,
                callback_function,
                qos
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')


        self.laplacian_matrix_publisher = self.create_publisher(
            Float64MultiArray,
            self.laplacian_topic_name, 
            qos
        )
        self.laplacian_timer = self.create_timer(0.1, self.publish_laplacian_matrix)


        self.lambda_gradient_publisher = self.create_publisher(
            Float64MultiArray,
            self.lambda_gradient_topic_name, 
            qos
        )
        self.gradient_timer = self.create_timer(0.1, self.publish_lambda_gradient)

    def publish_laplacian_matrix(self):
        if self.math_handler:
            laplacian_matrix = self.math_handler.Get_laplacian_matrix()
            if laplacian_matrix is None:
                return
            msg = numpy_matrix_to_float64multArray(laplacian_matrix)
            self.laplacian_matrix_publisher.publish(msg)

    def publish_lambda_gradient(self):
        if self.math_handler:
            lambda_gradient = self.math_handler.Get_gradient_vector()
            if lambda_gradient is None:
                return
            msg = numpy_matrix_to_float64multArray(lambda_gradient)
            self.lambda_gradient_publisher.publish(msg)


    

    def listener_callback(self, msg, robot_index):
        robot_name = self.robots_list[robot_index] 
        self.robots_instances[robot_name].Set_pose(msg)
        robot_instance_list = [self.robots_instances[robot] for robot in self.robots_list]
        self.math_handler.refresh_laplacian_matrix(robot_instance_list)

    def on_map_cb(self,msg):
        self.map_handler = MapHandler(
            map_data=msg.data,
            map_info=msg.info
        )
        self.map_handler.compute_map_distance()
        self.map_handler.generate_distances_colormap("/home/victor/projects/final_proj_MR/images/map.png")

        self.math_handler = MathHandler(self)

def main(args=None):
    rclpy.init(args=args)
    node = RobotsMathNode()
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