import rclpy
from rclpy.node import Node
from .RobotClass import RobotClass
import numpy as np
from geometry_msgs.msg import Twist, Pose
from .MathHandler import MatrixHandler
from .Ros2Utils import float64multArray_to_numpy_matrix
import cvxpy as cp
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from typing import Dict
from rclpy.qos import QoSProfile
from functools import partial
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool


EPSILON = 0.3
GAMMA = 3

class SingleRobotControllerNode(Node):
    def __init__(self):
        super().__init__("single_robot_controller_node")
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)


        self.declare_parameter("robots_list", [''], robot_list_descriptor)
        self.robots_list = self.get_parameter("robots_list").value
        self.robots_list = [] if self.robots_list == [''] else self.robots_list
        self.n_robots = len(self.robots_list)

        self.declare_parameter("robot_number", 0)
        self.robot_number = self.get_parameter("robot_number").value
        self.robot_name = f"robot{self.robot_number}"
        
        self.declare_parameter("robot_role", "conn")
        self.robot_role = self.get_parameter("robot_role").value


        self.n_robots = len(self.robots_list)

        self.declare_parameter("laplacian_topic_name", "laplacian_matrix")
        self.laplacian_topic_name = self.get_parameter("laplacian_topic_name").value

        self.declare_parameter("lambda_gradient_topic_name", "lambda2_gradient")
        self.lambda_gradient_topic_name = self.get_parameter("lambda_gradient_topic_name").value

        self.declare_parameter("ideal_cmd_vel_topic_name", "ideal_cmd_vel")
        self.ideal_cmd_vel_topic_name= self.get_parameter("ideal_cmd_vel_topic_name").value

        self.last_cmd_time = self.get_clock().now()

        self.subscriptions_dict_position = {}
        qos = QoSProfile(depth=10)


        self.robots_instances:Dict[str, RobotClass] = {}


        self.nav2_vel_vector = np.zeros((2,1))
        self.matrix_handler = MatrixHandler(self.n_robots)
        self.gradient_vector = np.zeros((self.n_robots*2,1))
        self.active = True

        '''
        ************************************************************************
        * Plublishers
        ************************************************************************
        '''
        self.vel_publisher = self.create_publisher(
            Twist,
            f"{self.robot_name}/cmd_vel"  , 
            qos
            )

        '''
        ************************************************************************
        * Timers creation
        ************************************************************************
        '''

        self.control_timer = self.create_timer(0.05, self.control_loop)


        '''
        ************************************************************************
        * Node Subscriptions
        ************************************************************************
        '''


        for i, robot_name in enumerate(self.robots_list):
            topic_name = f"{robot_name}/position"             
            self.robots_instances[robot_name] = RobotClass(robot_name)
            callback_function = partial(self.on_pose_cb, robot_index=i)
            self.subscriptions_dict_position[robot_name] = self.create_subscription(
                Pose,
                topic_name,
                callback_function,
                qos
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')


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



        topic_name = f"{self.robot_name}/{self.ideal_cmd_vel_topic_name}"
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            topic_name,
            self.on_cmd_vel_cb,
            qos
        )
        self.get_logger().info(f'Subscribed to: {topic_name}')


        # self.subscription_start = self.create_subscription(
        #     Bool, # Ou o tipo de mensagem 'generic' que você está usando
        #     "/startSimulation",
        #     self.toggle_callback,
        #     qos
        # )



    '''
    ****************************************************************************
    * Topic callbacks
    ****************************************************************************
    '''

    def on_pose_cb(self, msg,robot_index):
        robot_name = self.robots_list[robot_index] 
        self.robots_instances[robot_name].Set_pose(msg)


    def on_laplacian_cb(self,msg):
        laplacian_matrix = float64multArray_to_numpy_matrix(msg)
        self.matrix_handler.Set_laplacian_matrix(laplacian_matrix)

    def on_gradient_cb(self,msg):
        self.gradient_vector = float64multArray_to_numpy_matrix(msg)

    def on_cmd_vel_cb(self, msg: Twist):        
        # linear_velocity = msg.linear.x
        # angular_velocity = msg.angular.z

        # vx, vy = self.Get_robot_instance().Linear_velocity_to_xy(linear_velocity, angular_velocity, 0.15)
        

        # self.nav2_vel_vector[0, 0] = float(vx)
        # self.nav2_vel_vector[1, 0] = float(vy)
        self.nav2_vel_vector[0, 0] = msg.linear.x
        self.nav2_vel_vector[1, 0] = msg.linear.y
        self.last_cmd_time = self.get_clock().now()
            

    def toggle_callback(self, msg):
        self.active = not self.active

    '''
    ****************************************************************************
    * Periodic functions
    ****************************************************************************
    '''
    def control_loop(self):
        if (self.active):
            is_conn = (self.robot_role == "conn")
            has_input = np.any(np.abs(self.nav2_vel_vector) > 1e-4)
            
            if has_input or is_conn:
                real_velocities_vector = self.get_optimized_movement_vector(self.nav2_vel_vector)
                self.send_robot_velocity(real_velocities_vector)
            else:
                self.send_robot_velocity(np.zeros((2,1)))
            









    '''
    ****************************************************************************
    * Helpers
    ****************************************************************************
    '''
    def Get_robot_specifict_gradient_values(self):
        return self.gradient_vector[(self.robot_number-1)*2:(self.robot_number)*2]

    def Get_direction_vector_based_on_the_gradient(self,desired_mag,grad_vector):
        mag = np.linalg.norm(grad_vector)
        direction = grad_vector / (mag + 1e-6)
        u_guide = direction * desired_mag
        return u_guide

    def Get_barrier_val(self,gamma,lambda_2,epsilon):
        return - gamma * (lambda_2 - epsilon)

    def Get_lambda_projection(self,grad_vector,mov_vector):
        return (grad_vector.T @ mov_vector).item()

    def Get_robot_instance(self):
        return self.robots_instances[self.robot_name]

    def Check_vector_greater_than_threshold(self,vector,threshold):
        vx_ideal = vector[0]
        vy_ideal = vector[1]

        if abs(vx_ideal) < threshold and abs(vy_ideal) < threshold:
            return False
        
        return True


    



    def get_optimized_movement_vector(self, ideal_vector):
        
        max_vel = 0.5
        REAL_MAX_W = 1.5  
        L_POINT = 0.2     
        max_lateral_vel = REAL_MAX_W * L_POINT
        lambda_conn_threshold = 0.5

        grad_vector = self.Get_robot_specifict_gradient_values()
        lambda_2, _ = self.matrix_handler.Get_second_eingenvalue_and_eingenvector()
        conn_barrier_val = self.Get_barrier_val(GAMMA,lambda_2,EPSILON)
        projection = self.Get_lambda_projection(grad_vector,ideal_vector)
        collision_safe = self.get_collision_safe(1)
        
        if self.robot_role == "conn" and lambda_2 < lambda_conn_threshold:
            ideal_vector = self.Get_direction_vector_based_on_the_gradient(0.95,grad_vector)

        if projection >= conn_barrier_val and collision_safe:
            return ideal_vector
        
        

        u_final = cp.Variable((2, 1))
        delta = cp.Variable((1, 1), nonneg=True)

        cost_movement = cp.sum_squares(u_final - ideal_vector)
        cost_slack = 1e8 * cp.sum_squares(delta) 
        
        objective = cp.Minimize(cost_movement + cost_slack)
        
        constraints = []

        constraints.append(grad_vector.T @ u_final >= conn_barrier_val - delta)
        constraints.append(cp.abs(u_final) <= max_vel)

        s, c = self.Get_robot_instance().Get_yaw_sine_and_cos()
        # constraints.append(cp.abs(-u_final[0]*s + u_final[1]*c) <= max_lateral_vel)

        if self.robot_role == "task":
            if not self.Check_vector_greater_than_threshold(ideal_vector,1e-5):
                return np.zeros((2,1))

        positions = [self.robots_instances[robot_name].pose.position for robot_name in self.robots_list if robot_name!=self.robot_name]
        p_curr = self.Get_robot_instance().pose.position
        
        for p in positions:  
            dx =  p_curr.x - p.x
            dy =  p_curr.y - p.y
            dist = np.hypot(dx, dy)

            if dist < 1.5:
                if dist > 0.01:
                    nx = dx / dist
                    ny = dy / dist
                else:
                    nx, ny = 1.0, 0.0 

                n_vec = np.array([[nx, ny]]) 
                h = dist - 1
                constraints.append(n_vec @ u_final >= -1 * h)
        

        problem = cp.Problem(objective, constraints)

        try:
            problem.solve(solver=cp.OSQP, verbose=False)
            
            if u_final.value is not None:
                return u_final.value

        except Exception as e:
            self.get_logger().error(f"[OPTIMIZER_DEBUG] CVXPY Failed: {e}")

        return ideal_vector




    def send_robot_velocity(self,velocities_vector):
        msg = Twist()
        # v_global = velocities_vector[0,0]
        # w_global = velocities_vector[1,0]
        # v,w = self.Get_robot_instance().feedback_linearization_global_velocities_to_vw(v_global,w_global,0.15)
        # msg.linear.x = v
        # msg.angular.z = w
        msg.linear.x = float(velocities_vector[0, 0])
        msg.linear.y = float(velocities_vector[1, 0])
        self.vel_publisher.publish(msg)

    def get_collision_safe(self, safe_dist):
        positions = [self.robots_instances[robot_name].pose.position for robot_name in self.robots_list]
        min_dist_found = float('inf')   

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                p_i = positions[i]
                p_j = positions[j]
                d = np.hypot(p_i.x - p_j.x, p_i.y - p_j.y)
                if d < min_dist_found:
                    min_dist_found = d

        return min_dist_found > safe_dist




    




def main(args=None):
    rclpy.init(args=args)
    node = SingleRobotControllerNode()
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