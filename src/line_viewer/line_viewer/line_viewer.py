#!/usr/bin/env python3
'''
********************************************************************************
* imports
********************************************************************************
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# --- IMPORTS DO TF ---
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

import numpy as np

from .MarkerHandler import MarkerHandler
from .MapHandler import MapHandler
from .RobotPosHandler import RobotPosHandler
'''
********************************************************************************
* Defines
********************************************************************************
'''

LINE_SCALE = 0.05
LINE_ALPHA = 1.0
REFERENCE_FRAME = "map"
MARKER_NAMESPACE = "sight_marker"
CRITICAL_THRESHOLD = 0.1
MAX_SAFE_DIST_OBSTACLE = 1
MAX_DIST_ROBOTS = 7.5


'''
********************************************************************************
* Node Class
********************************************************************************
'''
class GazeboLinePublisher(Node):
    
    
    def __init__(self):
        super().__init__('gazebo_line_publisher')
        
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter("robot_list", [''], robot_list_descriptor)
        self.declare_parameter("publisher_node_name", "visualization_marker")
        self.declare_parameter("publisher_time_interval", 0.1)

        self.robot_list = self.get_parameter("robot_list").value
        self.publisher_node_name = self.get_parameter("publisher_node_name").value
        self.publisher_time_interval = self.get_parameter("publisher_time_interval").value

        self.robot_list = [] if self.robot_list == [''] else self.robot_list
            
        self.marker_handler = MarkerHandler(
            max_safe_distance=MAX_SAFE_DIST_OBSTACLE,
            min_safe_distance=CRITICAL_THRESHOLD,
            line_alpha=LINE_ALPHA,
            line_scale=LINE_SCALE
        )
        self.robot_position_handler = RobotPosHandler(self.robot_list,self)
        self.map_handler = None
        
        n_robots = len(self.robot_list)

        self.adjacency_matrix = np.zeros(n_robots,n_robots)
        self.degree_matrix = np.zeros(n_robots,n_robots)
        self.laplacian_matrix = np.zeros(n_robots,n_robots)

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.create_map_handler,
            QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.Init_marker_publisher(
            pub_time=self.publisher_time_interval,
            topic_name=self.publisher_node_name,
            pub_function=self.publish_lines
        )
        


    def create_map_handler(self,msg):
        self.map_handler = MapHandler(
            map_data=msg.data,
            map_info=msg.info
        )
        self.map_handler.compute_map_distance()
        self.map_handler.generate_distances_colormap("/home/victor/projects/final_proj_MR/images/map.png")

    def Init_marker_publisher(self, pub_time, topic_name, pub_function):
        qos = QoSProfile(depth=10)
        self.marker_pub = self.create_publisher(Marker, topic_name, qos)
        self.timer = self.create_timer(pub_time, pub_function)
        self.get_logger().info(f'TF Line Viewer Started | Update Rate: {1/pub_time:.1f} Hz')




    def publish_lines(self):
        current_positions = self.robot_position_handler.Get_all_robots_position(REFERENCE_FRAME)
        robots_found = list(current_positions.keys())
        
        for i, r1 in enumerate(robots_found):
            for j, r2 in enumerate(robots_found):
                if i >= j: continue
                self.publish_marker_update(
                    i,j,current_positions,robots_found
                )
                
    
    def publish_marker_update(self,i,j,current_positions,robots_found):
        robot1 = robots_found[i]
        robot2 = robots_found[j]
        
        p1 = current_positions[robot1]
        p2 = current_positions[robot2]

        robot_dist = np.hypot(p2.x-p1.x, p2.y-p1.y)
        min_dist = self.map_handler.get_line_min_dist_to_obstacle(p1,p2)
        score = self.calculate_connection_score(robot_dist,min_dist)

        rgb_color = self.marker_handler.get_marker_color(score)
        marker_line = self.marker_handler.create_marker(
            point1 = p1,
            point2 = p2,
            marker_id = hash(frozenset([robot1, robot2])) % 1000,
            rgb_color =rgb_color,
            ref_frame = REFERENCE_FRAME,
            namespace = MARKER_NAMESPACE,
            timestamp = self.get_clock().now().to_msg()
        )
        self.marker_pub.publish(marker_line)

    def update_laplacian_matrix(self,score,i,j):
        self.adjacency_matrix[i,j] = score
        self.adjacency_matrix[j,i] = score
        degree_vector = np.sum(self.adjacency_matrix, axis=1)
        self.degree_matrix = np.diag(degree_vector)
        self.laplacian_matrix = self.degree_matrix - self.laplacian_matrix



    def calculate_sight_score(self,min_obstacle_distance):
        return np.clip((min_obstacle_distance-CRITICAL_THRESHOLD)/(MAX_SAFE_DIST_OBSTACLE-CRITICAL_THRESHOLD),0,1)
        
    def calculate_distance_score(self,distance):
        return np.clip((MAX_DIST_ROBOTS - distance)/(MAX_DIST_ROBOTS),0,1)

    def calculate_connection_score(self,robot_dist,obstacle_dist):
        distance_score = self.calculate_distance_score(robot_dist)
        sight_score = self.calculate_sight_score(obstacle_dist)
        return distance_score*sight_score
        
'''
********************************************************************************
* Main function
********************************************************************************
'''

def main(args=None):
    rclpy.init(args=args)
    node = GazeboLinePublisher()
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