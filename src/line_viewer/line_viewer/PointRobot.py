import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Quaternion

class PointRobot:
    '''
********************************************************************************
* Class Constructor
********************************************************************************
    '''
    
    def __init__(self,x,y,yaw,task):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.task = task
        self.xSpeed = 0.05
        self.ySpeed = 0.0
        self.yawSpeed = 0.0


    '''
********************************************************************************
* Class dunder methods
********************************************************************************
    '''

    def __repr__(self) -> str:
        return f"Robot(x={self.x:.2f}, y={self.y:.2f})"

    '''
********************************************************************************
* Class Getters and Setters
********************************************************************************
    '''

    def SetX(self, x):
        self.x = x

    def GetX(self):
        return self.x

    def SetY(self, y):
        self.y = y

    def GetY(self):
        return self.y
    
    def SetYaw(self, yaw):
        self.yaw = yaw

    def GetYaw(self):
        return self.yaw

    def SetXSpeed(self, speed):
        self.xSpeed = speed

    def GetXSpeed(self):
        return self.xSpeed

    def SetYSpeed(self, speed):
        self.ySpeed = speed

    def GetYSpeed(self):
        return self.ySpeed
    
    def SetYawSpeed(self, speed):
        self.yawSpeed = speed

    def GetYawSpeed(self):
        return self.yawSpeed
        
    '''
********************************************************************************
* Class Interation functions
********************************************************************************
    '''

    def Step(self,timeInterval):
        self.x += self.xSpeed * timeInterval
        self.y += self.ySpeed * timeInterval
        self.yaw += self.yawSpeed * timeInterval
    '''
********************************************************************************
* Class Helpers
********************************************************************************
    '''
        



'''
********************************************************************************
* ROS2 Node
********************************************************************************
'''

class PointRobotNode(Node):
    def __init__(self):
        super().__init__('point_robot_sim')
        
        self.declare_parameter("xPos", 0.0)
        xPos = self.get_parameter("xPos").value

        self.declare_parameter("yPos", 0.0)
        yPos = self.get_parameter("yPos").value

        self.declare_parameter("yaw", 0.0)
        yaw = self.get_parameter("yaw").value

        self.declare_parameter("task", "conn")
        task = self.get_parameter("task").value

        self.init_x = xPos
        self.init_y = yPos

        self.robot = PointRobot(xPos, yPos, yaw, task)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.cmd_sub = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_callback, 
            10)

        self.dt = 0.05 
        self.timer = self.create_timer(self.dt, self.control_loop)

        ns = self.get_namespace().strip('/')
        self.odom_frame = f"{ns}/odom" if ns else "odom"
        self.base_frame = f"{ns}/base_link" if ns else "base_link"

    def cmd_callback(self, msg):
        self.robot.SetXSpeed(msg.linear.x)
        self.robot.SetYSpeed(msg.linear.y)
        self.robot.SetYawSpeed(msg.angular.z)


    def control_loop(self):
        self.get_logger().info("CONTROL LOOP")
        self.robot.Step(self.dt)
        current_time = self.get_clock().now().to_msg()
        q = self.yaw_to_quat(self.robot.yaw)

        # 1. TF Odom -> Base Link 
        # Aqui subtraímos a posição inicial para que o frame base_link 
        # se mova RELATIVO ao frame odom.
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # O PULO DO GATO:
        t.transform.translation.x = float(self.robot.x - self.init_x)
        t.transform.translation.y = float(self.robot.y - self.init_y)
        t.transform.rotation = q
        
        # 2. TF Map -> Odom (Fica parado na posição inicial)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.translation.x = float(self.init_x)
        map_to_odom.transform.translation.y = float(self.init_y)
        map_to_odom.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t, map_to_odom])
        
        # 3. Odometria (Nav2 espera a posição relativa ao frame odom)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.robot.x - self.init_x)
        odom.pose.pose.position.y = float(self.robot.y - self.init_y)
        odom.pose.pose.orientation = q
        # Velocidades são locais, então não mudam
        odom.twist.twist.linear.x = float(self.robot.xSpeed)
        odom.twist.twist.linear.y = float(self.robot.ySpeed)
        odom.twist.twist.angular.z = float(self.robot.yawSpeed)
        
        self.odom_pub.publish(odom)
    # Ver função pronta do ROS
    def yaw_to_quat(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))


def main(args=None):
    rclpy.init(args=args)
    node = PointRobotNode()
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