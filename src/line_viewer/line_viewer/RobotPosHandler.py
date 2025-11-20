from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point
import rclpy
from tf2_ros import TransformException


class RobotPosHandler:
    def __init__(self,robot_list,parent_node):
        self.robot_list = robot_list
        self.parent_node = parent_node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, parent_node)

    def get_robot_position(self, robot_name, reference_frame):
        target_frame = f"{robot_name}/base_link"
        source_frame = reference_frame

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

    def Get_all_robots_position(self,reference_frame):
        current_positions = {}
        for robot in self.robot_list:
            pos = self.get_robot_position(robot,reference_frame)
            if pos:
                current_positions[robot] = pos
        return current_positions