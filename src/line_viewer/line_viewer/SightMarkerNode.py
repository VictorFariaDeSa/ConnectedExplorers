import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA # Necessário para cores por vértice
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from .RobotClass import RobotClass
from geometry_msgs.msg import Pose
from functools import partial
from typing import Dict
from std_msgs.msg import Float64MultiArray
import numpy as np
from .Ros2Utils import float64multArray_to_numpy_matrix
import json

class SightMarkerNode(Node):
    def __init__(self):
        super().__init__('sight_marker_node')
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
        
        self.declare_parameter("robots_function_map", "{}")
        json_str = self.get_parameter('robots_function_map').get_parameter_value().string_value
        self.robots_functions = json.loads(json_str)

        self.laplacian_matrix = None
        self.subscriptions_dict = {}
        self.robots_instances:Dict[str, RobotClass] = {}

        qos = QoSProfile(depth=10)

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

        # NOVO Publisher para as ESFERAS
        self.sphere_publisher = self.create_publisher(
            Marker, 
            "robots_spheres", # Nome do novo tópico
            qos
        )
        
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        self.timer = self.create_timer(0.1, self.publish_markers)

    def laplacian_matrix_cb(self,msg):
        self.laplacian_matrix = float64multArray_to_numpy_matrix(msg)

    def listener_callback(self, msg, robot_index):
        robot_name = self.robots_list[robot_index] 
        self.robots_instances[robot_name].Set_pose(msg)

    def publish_markers(self):
        # Verificações de segurança
        if self.laplacian_matrix is None:
            return
        num_robots = len(self.robots_list)
        if self.laplacian_matrix.shape != (num_robots, num_robots):
            return

        now = self.get_clock().now().to_msg()

        # --- CONFIGURAÇÃO DO MARCADOR DE LINHAS ---
        line_marker = Marker()
        line_marker.header.frame_id = self.reference_frame
        line_marker.header.stamp = now
        line_marker.ns = "sight_lines"
        line_marker.id = 0 
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = float(self.line_scale)
        line_marker.pose.orientation.w = 1.0

        # --- CONFIGURAÇÃO DO MARCADOR DE ESFERAS ---
        sphere_marker = Marker()
        sphere_marker.header.frame_id = self.reference_frame
        sphere_marker.header.stamp = now
        sphere_marker.ns = "robot_bodies"
        sphere_marker.id = 1 
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.scale.x = 0.2 # Diâmetro da esfera
        sphere_marker.scale.y = 0.2
        sphere_marker.scale.z = 0.2
        sphere_marker.pose.orientation.w = 1.0

        # --- PREENCHIMENTO DOS DADOS ---
        for i, r_name_1 in enumerate(self.robots_list):
            r1 = self.robots_instances[r_name_1]
            p1 = r1.pose.position
            
            func = self.robots_functions.get(r_name_1, "default")
            
            sphere_color = ColorRGBA(a=1.0)
            if func == "task":
                sphere_color.r = 0.0; sphere_color.g = 1.0; sphere_color.b = 0.0 
            elif func == "conn":
                sphere_color.r = 0.0; sphere_color.g = 0.0; sphere_color.b = 1.0 
            else:
                sphere_color.r = 1.0; sphere_color.g = 1.0; sphere_color.b = 1.0 

            sphere_marker.points.append(p1)
            sphere_marker.colors.append(sphere_color)

            for j, r_name_2 in enumerate(self.robots_list):
                if i >= j:
                    continue

                r2 = self.robots_instances[r_name_2]
                score = self.laplacian_matrix[i, j]
                rgb = self.get_marker_color(score)

                color_msg = ColorRGBA(
                    r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(self.line_alpha)
                )

                # Adiciona par de pontos para a linha
                line_marker.points.append(p1)
                line_marker.colors.append(color_msg)
                
                p2 = r2.pose.position
                line_marker.points.append(p2)
                line_marker.colors.append(color_msg)

        # Publica em tópicos separados
        self.marker_publisher.publish(line_marker)
        self.sphere_publisher.publish(sphere_marker)


    def get_marker_color(self, score):
        score = abs(score)
        threshold = 1/(1+np.exp(-3*(0-1)))
        
        if score < threshold:
            rgb_color = (1.0, 0.0, 0.0)
        else:         
            r_val = 1.0 - score
            g_val = 1.0 
            b_val = 0.0
            r_val = max(0.0, min(1.0, r_val))
            rgb_color = (r_val, g_val, b_val)
            
        return rgb_color

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