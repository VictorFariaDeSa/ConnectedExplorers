# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster

# 1. Classe de Lógica Pura (Seguindo o exemplo)
class PointRobot:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def step(self, dt, vx, vy, vz, wz):
        # Integração de Euler simples
        self.x += vx * dt
        self.y += vy * dt
        self.z+= vz * dt
        self.yaw += wz * dt
        # Normalização do ângulo
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def __repr__(self) -> str:
        return f"Robot(x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.2f}°)"


# 2. Nó ROS 2 (Seguindo o exemplo)
class PointRobotNode(Node):
    def __init__(self):
        super().__init__(node_name='point_robot_sim')
        
        # --- Handle Parameters ---
        self.declare_parameter("xPos", 0.0)
        self.declare_parameter("yPos", 0.0)
        self.declare_parameter("zPos", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("frame_id","map")

        self.x = self.get_parameter("xPos").get_parameter_value().double_value
        self.y = self.get_parameter("yPos").get_parameter_value().double_value
        self.z = self.get_parameter("zPos").get_parameter_value().double_value
        self.yaw = self.get_parameter("yaw").get_parameter_value().double_value
        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        # Estado Interno
        self.robot = PointRobot(self.x, self.y, self.z, self.yaw)
        self.vx, self.vy,self.vz, self.wz = 0.0, 0.0, 0.0, 0.0

        # ROS 2 Infrastructure
        qos = QoSProfile(depth=10)
        topic_prefix = self.get_namespace()
        
        # Subscriber e Publisher (Nomes relativos para funcionar com namespaces)
        self.cmd_sub = self.create_subscription(Twist, f'{topic_prefix}/cmd_vel', self.cmd_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, f'{topic_prefix}/odom', qos)
        

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self.dt, self.update_physics)
        
        self.get_logger().info(f"Node {topic_prefix} iniciado em X:{self.x} Y:{self.y}")

    def cmd_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.wz = msg.angular.z

    def update_physics(self):
        # Atualiza a cinemática
        self.robot.step(self.dt, self.vx, self.vy, self.vz, self.wz)
        
        now = self.get_clock().now()
        self.broadcast_state(now)

    def broadcast_state(self, now):
        cy = math.cos(self.robot.yaw * 0.5)
        sy = math.sin(self.robot.yaw * 0.5)
        
        ns = self.get_namespace().strip('/')
        child_frame = f"{ns}/base_link" if ns else "base_link"

        # --- Publicar Odometria ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "map" # Link direto com o mapa como no exemplo
        odom.child_frame_id = child_frame
        
        odom.pose.pose.position.x = self.robot.x
        odom.pose.pose.position.y = self.robot.y
        odom.pose.pose.position.z = self.robot.z
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = self.vz
        odom.twist.twist.angular.z = self.wz
        
        self.odom_pub.publish(odom)


        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = child_frame
        
        t.transform.translation.x = self.robot.x
        t.transform.translation.y = self.robot.y
        t.transform.translation.z = self.robot.z
        t.transform.rotation.w = cy
        t.transform.rotation.z = sy
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PointRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()