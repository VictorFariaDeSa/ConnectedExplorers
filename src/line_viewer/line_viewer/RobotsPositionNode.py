import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point
from tf2_ros import TransformException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile

class RobotsPositionNode(Node):
    def __init__(self):
        super().__init__('gazebo_line_publisher')
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        
        self.declare_parameter("reference_frame", "map")
        self.reference_frame = self.get_parameter("reference_frame").value

        self.declare_parameter("robots_list", [''], robot_list_descriptor)
        self.robots_list = self.get_parameter("robots_list").value
        self.robots_list = [] if self.robots_list == [''] else self.robots_list

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=10)
        self.robot_publishers = {}

        for robot_name in self.robots_list:
            topic_name = f"{robot_name}/position"             
            self.robot_publishers[robot_name] = self.create_publisher(
                Point, 
                topic_name, 
                qos
            )
            self.get_logger().info(f'Publisher created for: {topic_name}')
        self.timer = self.create_timer(0.1, self.publish_all_positions)


    def publish_all_positions(self):
        for robot_name, publisher in self.robot_publishers.items():
            pos = self.get_robot_position(robot_name)
            if pos:
                publisher.publish(pos)

    def get_robot_position(self, robot_name):
        target_frame = f"{robot_name}/base_link"

        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame,
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


def main(args=None):
    rclpy.init(args=args)
    node = RobotsPositionNode()
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