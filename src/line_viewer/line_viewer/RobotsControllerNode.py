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
import json

class RobotsControllerNode(Node):
    def __init__(self):
        super().__init__('robots_controller')
        robot_list_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)

        '''
        ************************************************************************
        * Parameters declaration
        ************************************************************************
        '''

        self.declare_parameter("robots_list", [''], robot_list_descriptor)
        self.robots_list = self.get_parameter("robots_list").value
        self.robots_list = [] if self.robots_list == [''] else self.robots_list

        self.declare_parameter("laplacian_topic_name", "laplacian_matrix")
        self.laplacian_topic_name = self.get_parameter("laplacian_topic_name").value

        self.declare_parameter("lambda_gradient_topic_name", "lambda2_gradient")
        self.lambda_gradient_topic_name = self.get_parameter("lambda_gradient_topic_name").value

        self.declare_parameter("ideal_cmd_vel_topic_name", "ideal_cmd_vel")
        self.ideal_cmd_vel_topic_name= self.get_parameter("ideal_cmd_vel_topic_name").value

        self.declare_parameter("robots_function_map", "{}")
        json_str = self.get_parameter('robots_function_map').get_parameter_value().string_value
        self.robots_functions = json.loads(json_str)

        self.epsilon = 0.3
        self.gamma = 3

        self.publishers_dict = {}
        self.subscriptions_dict_cmd_vel = {}
        self.subscriptions_dict_position = {}

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.robots_instances:Dict[str, RobotClass] = {}

        self.n_robots = len(self.robots_list)
        self.matrix_handler = MatrixHandler(self.n_robots)
        self.gradient_vector = np.zeros((self.n_robots*2,1))
        self.nav2_vel_vector = np.zeros((self.n_robots*2,1))


        qos = QoSProfile(depth=10)


        '''
        ************************************************************************
        * Node Subscriptions
        ************************************************************************
        '''

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


        '''
        ************************************************************************
        * Node Publishers
        ************************************************************************
        '''

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
        vx,vy = curr_robot.Linear_velocity_to_xy(linear_velocity,angular_velocity,0.15)
        self.nav2_vel_vector[robot_index*2] = vx
        self.nav2_vel_vector[robot_index*2+1] = vy


    def control_loop(self):
        has_active_input = np.any(np.abs(self.nav2_vel_vector) > 1e-4)        
        if has_active_input:
            real_velocities_vector = self.get_optimized_movement_vector(self.nav2_vel_vector)
            self.send_robot_velocity(real_velocities_vector)


    def send_robot_velocity(self,velocities_vector):
        for i,robot_name in enumerate(self.robots_list):
            robot = self.robots_instances[robot_name]
            publisher = self.publishers_dict[robot_name]
            v_global = velocities_vector[2*i, 0]
            w_global = velocities_vector[2*i+1, 0]
            v,w = robot.feedback_linearization_global_velocities_to_vw(v_global,w_global,0.15)
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            publisher.publish(msg)



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
    
    def get_optimized_movement_vector(self, ideal_vector):
        V_rec = 0.95 
        
        lambda_2, _ = self.matrix_handler.Get_second_eingenvalue_and_eingenvector()
        conn_barrier_val = - self.gamma * (lambda_2 - self.epsilon)

        projection = (self.gradient_vector.T @ ideal_vector).item()
        
        collision_safe = self.get_collision_safe(1)
        
        # Se estiver seguro e projetando positivamente, retorna o ideal
        if projection >= conn_barrier_val and collision_safe:
            return ideal_vector
        
        ideal_col = ideal_vector.reshape(-1, 1).copy()
        n_vars = ideal_col.shape[0]
        
        # Pesos (Conn vs Task)
        weights_diag = np.ones(n_vars)
        for i,robot_name in enumerate(self.robots_list):
            idx = i*2 
            if self.robots_functions[robot_name] == "conn":
                weights_diag[idx : idx + 2] = 0.01              
            else:
                weights_diag[idx : idx + 2] = 10.0 
        
        # Lógica de guia para conectividade (se necessário)
        if (projection < conn_barrier_val):
            grad_vec_1d = self.gradient_vector.T[0]
            
            idx_winner = -1
            mag_winner = 0.0
            for i,robot_name in enumerate(self.robots_list):
                idx = i*2 
                if self.robots_functions[robot_name] == "conn":            
                    mag = np.linalg.norm(grad_vec_1d[idx : idx + 2])
                    if mag > mag_winner:
                        idx_winner = idx
                        mag_winner = mag          

            if idx_winner != -1:
                grad_dir = grad_vec_1d[idx_winner : idx_winner + 2]
                direction = grad_dir / mag_winner
                u_guide = direction * V_rec
                ideal_col[idx_winner : idx_winner + 2, 0] = u_guide

        # Configuração do problema de otimização
        W = np.diag(weights_diag)

        u_final = cp.Variable((n_vars, 1))
        delta = cp.Variable((1, 1), nonneg=True)

        cost_movement = cp.quad_form(u_final - ideal_col, W)
        cost_slack = 1e8 * cp.sum_squares(delta) 
        
        objective = cp.Minimize(cost_movement + cost_slack)
        
        max_vel = 0.5
        REAL_MAX_W = 1.5  
        L_POINT = 0.2     
        max_lateral_vel = REAL_MAX_W * L_POINT
        
        constraints = [
            self.gradient_vector.T @ u_final >= conn_barrier_val - delta,
            cp.abs(u_final) <= max_vel
        ]

        # --- NOVA LÓGICA AQUI ---
        for i, robot in enumerate(self.robots_list):
            # 1. Restrição de Velocidade Lateral (Original)
            yaw = self.robots_instances[robot].yaw
            s, c = np.sin(yaw), np.cos(yaw)
            constraints.append(cp.abs(-u_final[2*i,0]*s + u_final[2*i+1,0]*c) <= max_lateral_vel)

            # 2. Restrição de Parada para Robôs "Task" (Nova)
            if self.robots_functions[robot] == "task":
                # Pega a velocidade ideal de entrada para este robô
                vx_ideal = ideal_col[2*i, 0]
                vy_ideal = ideal_col[2*i+1, 0]

                # Verifica se a entrada é zero (usando pequena margem para float)
                if abs(vx_ideal) < 1e-5 and abs(vy_ideal) < 1e-5:
                    # Força a saída do otimizador a ser zero
                    constraints.append(u_final[2*i, 0] == 0)
                    constraints.append(u_final[2*i+1, 0] == 0)

        
        # Restrições de Colisão (Inter-agentes)
        positions = [self.robots_instances[robot_name].pose.position for robot_name in self.robots_list]
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)): 
                p_i = positions[i]
                p_j = positions[j]
                
                dx = p_i.x - p_j.x
                dy = p_i.y - p_j.y
                
                dist = np.hypot(dx, dy)

                if dist < 1.5:
                    if dist > 0.01:
                        nx = dx / dist
                        ny = dy / dist
                    else:
                        nx, ny = 1.0, 0.0 

                    n_vec = np.array([[nx, ny]]) 

                    ui_var = u_final[2*i : 2*i+2]
                    uj_var = u_final[2*j : 2*j+2]

                    h = dist - 1
                    constraints.append(
                        n_vec @ (ui_var - uj_var) >= -1 * h
                    )

        problem = cp.Problem(objective, constraints)

        try:
            problem.solve(solver=cp.OSQP, verbose=False)
            
            if u_final.value is not None:
                return u_final.value
            else:
                self.get_logger().warn("Solver Inviável (Retornando Ideal)")

        except Exception as e:
            self.get_logger().error(f"CVXPY Failed: {e}")

        return ideal_vector
    



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