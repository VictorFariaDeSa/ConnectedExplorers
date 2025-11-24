import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from .MathHandler import MatrixHandler
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from .Ros2Utils import float64multArray_to_numpy_matrix
import numpy as np
from geometry_msgs.msg import Twist, Pose
from .RobotClass import RobotClass
from typing import Dict
from functools import partial
import cvxpy as cp

class RobotsControllerNode(Node):
    def __init__(self):
        super().__init__('robots_controller')
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        
        self.declare_parameter("robots_list", [''], robot_list_descriptor)
        self.robots_list = self.get_parameter("robots_list").value
        self.robots_list = [] if self.robots_list == [''] else self.robots_list

        self.declare_parameter("laplacian_topic_name", "laplacian_matrix")
        self.laplacian_topic_name = self.get_parameter("laplacian_topic_name").value

        self.declare_parameter("lambda_gradient_topic_name", "lambda2_gradient")
        self.lambda_gradient_topic_name = self.get_parameter("lambda_gradient_topic_name").value

        self.declare_parameter("ideal_cmd_vel_topic_name", "ideal_cmd_vel")
        self.ideal_cmd_vel_topic_name= self.get_parameter("ideal_cmd_vel_topic_name").value

        self.epsilon = 0.3
        self.gamma = 0.3

        self.publishers_dict = {}
        self.subscriptions_dict_cmd_vel = {}
        self.subscriptions_dict_position = {}


        self.robots_instances:Dict[str, RobotClass] = {}

        self.n_robots = len(self.robots_list)
        self.matrix_handler = MatrixHandler(self.n_robots)
        self.gradient_vector = np.zeros((self.n_robots*2,1))

        qos = QoSProfile(depth=10)

        self.laplacian_matrix_subscriber = self.create_subscription(
            Float64MultiArray,
            self.laplacian_topic_name ,
            self.on_laplacian_cb,
            qos
        )
        self.get_logger().info(f'Subscribed to: {self.laplacian_topic_name }')

    
        self.lambda2_gradient_subscriber = self.create_subscription(
            Float64MultiArray,
            self.lambda_gradient_topic_name,
            self.on_gradient_cb,
            qos
        )
        self.get_logger().info(f'Subscribed to: {self.lambda_gradient_topic_name}')
    

        for i, robot_name in enumerate(self.robots_list):
            topic_name = f"{robot_name}/position"             
            self.robots_instances[robot_name] = RobotClass(robot_name)
            callback_function = partial(self.listener_callback, robot_index=i)
            self.subscriptions_dict_position[robot_name] = self.create_subscription(
                Pose,
                topic_name,
                callback_function,
                qos
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')



        for i, robot_name in enumerate(self.robots_list):
            topic_name = f"{robot_name}/{self.ideal_cmd_vel_topic_name}"             
            callback_function = partial(self.on_cmd_vel, robot_index=i)
            self.subscriptions_dict_cmd_vel[robot_name] = self.create_subscription(
                Twist,
                topic_name,
                callback_function,
                qos
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')


        for i, robot_name in enumerate(self.robots_list):
            topic_name = f"{robot_name}/cmd_vel"  
            self.publishers_dict[robot_name] =  self.create_publisher(
            Twist,
            topic_name, 
            qos
            )
            self.get_logger().info(f'Publisher to: {topic_name} created')

    def on_cmd_vel(self,msg:Twist,robot_index):
        robot_name = self.robots_list[robot_index]
        
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        curr_robot = self.robots_instances[robot_name]
        vx,vy = curr_robot.Linear_velocity_to_xy(linear_velocity,angular_velocity,0.15+0.15)
        self.velocities_vector = np.zeros((self.n_robots*2,1))
        self.velocities_vector[robot_index*2] = vx
        self.velocities_vector[robot_index*2+1] = vy
        real_velocities_vector = self.get_optimized_movement_vector(self.velocities_vector)
        # self.send_robot_velocity(self.velocities_vector)
        self.send_robot_velocity(real_velocities_vector)


    def send_robot_velocity(self,velocities_vector):
        for i,robot_name in enumerate(self.robots_list):
            robot = self.robots_instances[robot_name]
            publisher = self.publishers_dict[robot_name]
            v_global = velocities_vector[2*i, 0]
            w_global = velocities_vector[2*i+1, 0]
            v,w = robot.feedback_linearization_global_velocities_to_vw(v_global,w_global,0.15+0.15)
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            publisher.publish(msg)



    def get_optimized_movement_vector(self,ideal_vector):
        lambda_2,_ = self.matrix_handler.Get_second_eingenvalue_and_eingenvector()
        barrier_val = - self.gamma * (lambda_2-self.epsilon)

        projection = (self.gradient_vector.T @ ideal_vector).item()
        if projection > barrier_val:
            return ideal_vector
        n_vars = ideal_vector.shape[0]

        u = cp.Variable((n_vars, 1))
        objective = cp.Minimize(cp.sum_squares(u - ideal_vector))
        max_vel = 5
        constraints = [
            self.gradient_vector.T @ u >= barrier_val,
            cp.abs(u) <= max_vel
        ]
        problem = cp.Problem(objective, constraints)
    
        try:
            problem.solve(solver=cp.OSQP, verbose=False)
            
            if u.value is not None:
                return u.value
                
        except Exception as e:
            self.get_logger().error(f"CVXPY Failed: {e}")

        return np.zeros_like(ideal_vector)






    def listener_callback(self, msg, robot_index):
        robot_name = self.robots_list[robot_index] 
        self.robots_instances[robot_name].Set_pose(msg)



    def on_laplacian_cb(self,msg):
        laplacian_matrix = float64multArray_to_numpy_matrix(msg)
        self.matrix_handler.Set_laplacian_matrix(laplacian_matrix)

    def on_gradient_cb(self,msg):
        self.gradient_vector = float64multArray_to_numpy_matrix(msg)






def main(args=None):
    rclpy.init(args=args)
    node = RobotsControllerNode()
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