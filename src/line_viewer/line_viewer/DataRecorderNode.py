import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile
import os
import csv
import time

class RobotsMathNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')


        self.create_subscription(
                Float64,
                '/fiedler_value',
                self.on_fiedler_cb,
                QoSProfile(depth=10)
            )


    def on_fiedler_cb(self, msg):
        dir_path = "/home/victor/projects/final_proj_MR/data"
        file_name = "fiedler_log.csv"
        full_path = os.path.join(dir_path, file_name)

        os.makedirs(dir_path, exist_ok=True)

        file_exists = os.path.isfile(full_path)

        with open(full_path, mode='a', newline='') as f:
            writer = csv.writer(f)

            if not file_exists:
                writer.writerow(['timestamp', 'fiedler_value'])
            current_time = time.time() 
            writer.writerow([current_time, msg.data])
            


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