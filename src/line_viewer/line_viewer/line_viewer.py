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
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# --- IMPORTS DO TF ---
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from nav_msgs.msg import OccupancyGrid

'''
********************************************************************************
* Defines
********************************************************************************
'''

LINE_SCALE = 0.05 # Fio fino
LINE_ALPHA = 1.0
REFERENCE_FRAME = "map"
MARKER_NAMESPACE = "sight_marker"



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
            

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.map_data = None
        self.map_info = None

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.Init_marker_publisher(
            pub_time=self.publisher_time_interval,
            topic_name=self.publisher_node_name,
            pub_function=self.publish_lines
        )
        

    def Init_marker_publisher(self, pub_time, topic_name, pub_function):
        qos = QoSProfile(depth=10)
        self.marker_pub = self.create_publisher(Marker, topic_name, qos)
        self.timer = self.create_timer(pub_time, pub_function)
        self.get_logger().info(f'TF Line Viewer Started | Update Rate: {1/pub_time:.1f} Hz')


    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info("Mapa received")

    def world_to_grid(self, world_point):
        if self.map_info is None:
            return None
            
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        width = self.map_info.width
        height = self.map_info.height

        map_x = int((world_point.x - origin_x) / resolution)
        map_y = int((world_point.y - origin_y) / resolution)

        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return None
            
        return (map_x, map_y)

    def is_cell_occupied(self, grid_x, grid_y):
        """Verifica se uma célula específica é parede"""
        index = grid_y * self.map_info.width + grid_x
        cell_value = self.map_data[index]
        if cell_value > 50 or cell_value == -1:
            return True
        return False
    
    def check_line_of_sight(self, p1, p2):
        """
        Retorna False se houver uma parede entre p1 e p2.
        Retorna True se estiver livre.
        """
        if self.map_data is None:
            return True 

        start = self.world_to_grid(p1)
        end = self.world_to_grid(p2)

        if start is None or end is None:
            return False 

        x0, y0 = start
        x1, y1 = end

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                if self.is_cell_occupied(x, y):
                    return False 
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if self.is_cell_occupied(x, y):
                    return False 
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        return True 

    def get_robot_position(self, robot_name):
        target_frame = f"{robot_name}/base_link"
        source_frame = REFERENCE_FRAME

        try:
            t = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time()
            )
            
            p = Point()
            p.x = t.transform.translation.x
            p.y = t.transform.translation.y
            p.z = t.transform.translation.z
            return p

        except TransformException:
            return None

    def create_marker(self, point1, point2, marker_id, rgb_color):
        marker = Marker()
        marker.header.frame_id = REFERENCE_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = MARKER_NAMESPACE
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.color.r, marker.color.g, marker.color.b = rgb_color
        marker.color.a = float(LINE_ALPHA)
        marker.scale.x = float(LINE_SCALE)
        
        marker.points = [point1, point2]
        return marker



    def Get_all_robots_position(self):
        current_positions = {}
        for robot in self.robot_list:
            pos = self.get_robot_position(robot)
            if pos:
                current_positions[robot] = pos
        return current_positions

    def publish_lines(self):
        current_positions = self.Get_all_robots_position()
        robots_found = list(current_positions.keys())
        
        for i, r1 in enumerate(robots_found):
            for j, r2 in enumerate(robots_found):
                if i >= j: continue

                p1 = current_positions[r1]
                p2 = current_positions[r2]
                
                marker_id = hash(frozenset([r1, r2])) % 1000
                visible = self.check_line_of_sight(p1,p2)
                rgb_color = (0.0, 1.0, 0.0) if visible else (1.0, 0.0, 0.0)
                marker = self.create_marker(p1, p2, marker_id, rgb_color)
                self.marker_pub.publish(marker)



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