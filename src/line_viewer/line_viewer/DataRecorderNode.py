import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile
import os
import csv
import time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from functools import partial



TIME = 0.5

class RobotsMathNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        
        self.declare_parameter("robots_list", [''], robot_list_descriptor)
        self.robots_list = self.get_parameter("robots_list").value
        self.robots_list = [] if self.robots_list == [''] else self.robots_list

        self.robots_pose_list = {}
        self.fidler_value = None

        for robot_name in self.robots_list:        
            callback_function = partial(self.pose_callback, robot_name = robot_name)
            topic_name = f"{robot_name}/position"  
            self.create_subscription(
                Pose,
                topic_name,
                callback_function,
                QoSProfile(depth=10),
            )
        
        self.create_subscription(
                Float64,
                '/fiedler_value',
                self.on_fiedler_cb,
                QoSProfile(depth=10)
            )
        

        self.position_timer = self.create_timer(TIME, self.write_poses)
        self.fiedler_timer = self.create_timer(TIME, self.write_fiedler)



    def write_poses(self):
        if not self.robots_list:
            return

        dir_path = "/home/victor/projects/ConnectedExplorers/data"
        file_name = "poses_log.csv"
        full_path = os.path.join(dir_path, file_name)

        os.makedirs(dir_path, exist_ok=True)

        file_exists = os.path.isfile(full_path)

        with open(full_path, mode='a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                header = ['timestamp']
                for robot_name in self.robots_list:
                    header.append(f'{robot_name}_x')
                    header.append(f'{robot_name}_y')
                    header.append(f'{robot_name}_z')
                writer.writerow(header)

            current_time = time.time() 
            row = [current_time]
            
            for robot_name in self.robots_list:
                pose = self.robots_pose_list.get(robot_name)
                
                if pose:
                    row.append(pose.position.x)
                    row.append(pose.position.y)
                    row.append(pose.position.z)
                else:
                    row.append('NaN')
                    row.append('NaN')
                    row.append('NaN')
            
            writer.writerow(row)

    
    def write_fiedler(self):
        dir_path = "/home/victor/projects/ConnectedExplorers/data"
        file_name = "fiedler_log.csv"
        full_path = os.path.join(dir_path, file_name)

        os.makedirs(dir_path, exist_ok=True)

        file_exists = os.path.isfile(full_path)

        with open(full_path, mode='a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'fiedler_value'])
            current_time = time.time() 

            if self.fidler_value:
                value = self.fidler_value
            else:
                value = "NaN"
            writer.writerow([current_time, value])
    



    def pose_callback(self, msg, robot_name):
        self.robots_pose_list[robot_name] = msg





    def on_fiedler_cb(self, msg):
        self.fidler_value = msg.data
            


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