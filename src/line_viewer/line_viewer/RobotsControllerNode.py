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
        vx,vy = curr_robot.Linear_velocity_to_xy(linear_velocity,angular_velocity,0.15)
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
            v,w = robot.feedback_linearization_global_velocities_to_vw(v_global,w_global,0.15)
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            publisher.publish(msg)





    def get_optimized_movement_vector(self, ideal_vector):
        # --- VARIÁVEIS DE CONFIGURAÇÃO ---
        V_rec = 1 * 0.95 
        
        # --- 1. CHECAGEM DE SEGURANÇA E BARREIRA ---
        lambda_2, _ = self.matrix_handler.Get_second_eingenvalue_and_eingenvector()
        barrier_val = - self.gamma * (lambda_2 - self.epsilon)

        projection = (self.gradient_vector.T @ ideal_vector).item()
        if projection >= barrier_val:
            return ideal_vector

        # --- 2. PREPARAÇÃO DA RECUPERAÇÃO (LÓGICA REALM) ---
        
        ideal_col = ideal_vector.reshape(-1, 1).copy()
        n_vars = ideal_col.shape[0]

        # Extrai gradientes auxiliares
        grad_vec_1d = self.gradient_vector.T[0]
        grad_R2 = grad_vec_1d[2:4] # [vx2, vy2]
        grad_R3 = grad_vec_1d[4:6] # [vx3, vy3]

        # Calcula magnitudes (Urgência Bruta)
        mag_R2 = np.linalg.norm(grad_R2)
        mag_R3 = np.linalg.norm(grad_R3)
        
        # [REALM] Implementação da Lógica de Fusão de Urgência
        # Em vez de um "winner-takes-all", calculamos pesos de colaboração.
        # Isso evita que um robô com gradiente fraco (devido à distância) seja ignorado
        # se ele for topologicamente importante.
        
        # Constante de fusão 'c' sugerida pelo artigo (ajuste conforme escala do mapa)
        c_fusion = 1.0 

        # Pesos de Fusão (Baseado na Eq. 6 do artigo Realm)
        # O robô com MENOR magnitude (mais seguro/estável) ganha peso para ajudar o mais urgente
        denom = mag_R2 + mag_R3 + 2 * c_fusion
        
        if denom > 1e-6:
            # Peso cruzado: A urgência de R3 impulsiona R2, e vice-versa
            weight_R2 = (mag_R3 + c_fusion) / denom
            weight_R3 = (mag_R2 + c_fusion) / denom
        else:
            weight_R2 = 0.5
            weight_R3 = 0.5

        # --- INJEÇÃO DE VETOR GUIA COLABORATIVO ---
        
        # Injeta para R2
        if mag_R2 > 1e-6:
            # Direção do gradiente original
            dir_R2 = grad_R2 / mag_R2
            # Força escalada pelo peso de fusão Realm
            # Se R3 estiver crítico, weight_R2 aumenta, forçando R2 a se mover
            u_guide_R2 = dir_R2 * (V_rec * weight_R2 * 2.0) # *2 para normalizar a escala
            ideal_col[2:4, 0] = u_guide_R2

        # Injeta para R3
        if mag_R3 > 1e-6:
            dir_R3 = grad_R3 / mag_R3
            u_guide_R3 = dir_R3 * (V_rec * weight_R3 * 2.0)
            ideal_col[4:6, 0] = u_guide_R3

        # --- 3. PESOS (W) ---
        weights_diag = np.ones(n_vars)
        weights_diag[0:2] = 10.0      # Prioridade Líder
        weights_diag[2:] = 0.01      # Prioridade Auxiliares
        W = np.diag(weights_diag)

        # --- 4. CONFIGURAÇÃO DO SOLVER QP ---
        u_final = cp.Variable((n_vars, 1))
        
        cost_movement = cp.quad_form(u_final - ideal_col, W)
        objective = cp.Minimize(cost_movement)
        
        # --- 5. RESTRIÇÕES ---
        max_vel = 0.5
        REAL_MAX_W = 1.5  
        L_POINT = 0.2     
        max_lateral_vel = REAL_MAX_W * L_POINT
        
        # Yaw e Sen/Cos
        yaw_r1 = self.robots_instances["robot1"].yaw
        yaw_r2 = self.robots_instances["robot2"].yaw
        yaw_r3 = self.robots_instances["robot3"].yaw

        s1, c1 = np.sin(yaw_r1), np.cos(yaw_r1)
        s2, c2 = np.sin(yaw_r2), np.cos(yaw_r2)
        s3, c3 = np.sin(yaw_r3), np.cos(yaw_r3)
        
        constraints = [
            self.gradient_vector.T @ u_final >= barrier_val,
            cp.abs(u_final) <= max_vel,
            cp.abs(-u_final[0,0]*s1 + u_final[1,0]*c1) <= max_lateral_vel,
            cp.abs(-u_final[2,0]*s2 + u_final[3,0]*c2) <= max_lateral_vel,
            cp.abs(-u_final[4,0]*s3 + u_final[5,0]*c3) <= max_lateral_vel
        ]
        
        # --- 6. SOLVE ---
        problem = cp.Problem(objective, constraints)

        try:
            problem.solve(solver=cp.OSQP, verbose=False)
            if u_final.value is not None:
                return u_final.value
            else:
                self.get_logger().warn("Solver Inviável com Hard CBF.")
        except Exception as e:
            self.get_logger().error(f"CVXPY Failed: {e}")

        return ideal_vector

    # def get_optimized_movement_vector(self, ideal_vector):
    #     lambda_2, _ = self.matrix_handler.Get_second_eingenvalue_and_eingenvector()
    #     barrier_val = - self.gamma * (lambda_2 - self.epsilon)

    #     projection = (self.gradient_vector.T @ ideal_vector).item()
        
    #     # Otimização: se já satisfaz, retorna o original sem gastar tempo de solver
    #     if projection > barrier_val:
    #         return ideal_vector

    #     # --- 1. Preparação (Sem achatar, mantendo matriz coluna) ---
    #     ideal_col = ideal_vector.reshape(-1, 1)
    #     n_vars = ideal_col.shape[0]

    #     # --- 2. Pesos (A "Inteligência" do Soft Lock) ---
    #     weights_diag = np.ones(n_vars)
        
    #     # Robô 1: Peso ALTO (10.000). 
    #     # Isso força o solver a ficar COLADO no vetor do Nav2.
    #     # Ele só vai desviar (mudar angulo) se os outros robôs não derem conta.
    #     weights_diag[0:2] = 10.0 
        
    #     # Outros Robôs: Peso BAIXO (0.01).
    #     # É barato mover eles. Eles serão os primeiros a serem "sacrificados" para a barreira.
    #     weights_diag[2:] = 0.01
        
    #     W = np.diag(weights_diag)




    #     yaw_r1 = self.robots_instances["robot1"].yaw
    #     yaw_r2 = self.robots_instances["robot2"].yaw
    #     yaw_r3 = self.robots_instances["robot3"].yaw

    #     s1, c1 = np.sin(yaw_r1), np.cos(yaw_r1)
    #     s2, c2 = np.sin(yaw_r2), np.cos(yaw_r2)
    #     s3, c3 = np.sin(yaw_r3), np.cos(yaw_r3)

    #     # u_final representa todo mundo [vx1, vy1, vx2, vy2, ...]
    #     u_final = cp.Variable((n_vars, 1))
        
    #     # Variável de folga (IMPEDE O CONGELAMENTO)
    #     delta = cp.Variable((1, 1), nonneg=True)

    #     # --- 4. Objetivo ---
    #     # Minimiza: (Diferença para o Ideal) + (Penalidade do Slack)
    #     # Graças ao W, a diferença do R1 pesa muito mais que a dos outros.
    #     cost_movement = cp.quad_form(u_final - ideal_col, W)
    #     cost_slack = 1e9 * cp.sum_squares(delta) # Peso 1 bilhão para evitar usar o slack
        
    #     objective = cp.Minimize(cost_movement + cost_slack)
        
    #     max_vel = 0.5
    #     REAL_MAX_W = 1.5  
    #     L_POINT = 0.2     
    #     max_lateral_vel = REAL_MAX_W * L_POINT
    #     constraints = [
    #         # Barreira com Slack (A "Válvula de Escape")
    #         self.gradient_vector.T @ u_final >= barrier_val - delta,
            
    #         # Limite Físico para TODOS
    #         cp.abs(u_final) <= max_vel,

    #         cp.abs(-u_final[0,0]*s1 + u_final[1,0]*c1) <= max_lateral_vel,
    #         cp.abs(-u_final[2,0]*s2 + u_final[3,0]*c2) <= max_lateral_vel,
    #         cp.abs(-u_final[4,0]*s3 + u_final[5,0]*c3) <= max_lateral_vel
    #     ]
        
    #     problem = cp.Problem(objective, constraints)
    
    #     try:
    #         problem.solve(solver=cp.OSQP, verbose=False)
            
    #         if u_final.value is not None:
    #             # Retorna no formato original (Matriz Coluna)
    #             return u_final.value
    #         else:
    #             self.get_logger().warn("Solver Inviável")

    #     except Exception as e:
    #         self.get_logger().error(f"CVXPY Failed: {e}")

    #     return np.zeros_like(ideal_vector)





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