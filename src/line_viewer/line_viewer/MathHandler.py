
import numpy as np
from .RobotClass import RobotClass
from .MapHandler import MapHandler, Cell, GridLine
from typing import TYPE_CHECKING, List

if TYPE_CHECKING:
    from .RobotsMathNode import RobotsMathNode



class MatrixHandler:
    def __init__(self,n_robots):
        self.adjacency_matrix = np.zeros((n_robots,n_robots))
        self.degree_matrix = np.zeros((n_robots,n_robots))
        self.laplacian_matrix = np.zeros((n_robots,n_robots))

    def Update_laplacian_matrix(self,score,i,j):
        self.adjacency_matrix[i,j] = score
        self.adjacency_matrix[j,i] = score
        degree_vector = np.sum(self.adjacency_matrix, axis=1)
        self.degree_matrix = np.diag(degree_vector)
        self.laplacian_matrix = self.degree_matrix - self.adjacency_matrix

    def Get_second_eingenvalue_and_eingenvector(self):
        eigenvalues, eigenvectors = np.linalg.eigh(self.laplacian_matrix)
        lambda_2 = eigenvalues[1]
        v_2 = eigenvectors[:, 1]
        return lambda_2, v_2
    
    def Get_laplacian_matrix(self):
        return self.laplacian_matrix


class MathHandler:
    def __init__(self,
        parent:'RobotsMathNode'
    ):
        self.n_nodes = len(parent.robots_list)
        self.matrix_handler = MatrixHandler(self.n_nodes)
        self.max_robots_dist = parent.max_robots_dist
        self.map_handler = parent.map_handler
        self.min_dist_to_wall = parent.min_dist_to_wall
        self.max_dist_to_wall = parent.max_dist_to_wall
        self.parent = parent


    def Update_laplacian_matrix(self,score,i,j):
        return self.matrix_handler.Update_laplacian_matrix(score,i,j)
    
    def get_second_eingenvalue_and_eingenvector(self):
        return self.matrix_handler.Get_second_eingenvalue_and_eingenvector()

    def Get_laplacian_matrix(self):
        return self.matrix_handler.Get_laplacian_matrix()

    def refresh_laplacian_matrix(self,robots_instance_list):
        for i, r1 in enumerate(robots_instance_list):
            for j, r2 in enumerate(robots_instance_list):
                if i >= j:continue
                score = self.calculate_connection_score(r1,r2)
                self.Update_laplacian_matrix(score,i,j)

    def calculate_distance_score(self,r1:RobotClass, r2:RobotClass):
        distance = r1.Get_distance_to(r2)
        return np.clip((self.max_robots_dist - distance)/(self.max_robots_dist),0,1)
    
    def calculate_sight_score(self,r1:RobotClass, r2:RobotClass):
        line = self.map_handler.Get_line_between_robots(r1,r2)
        min_obstacle_dist,_ = self.map_handler.get_line_min_dist_to_obstacle(line)
        score = (min_obstacle_dist-self.min_dist_to_wall)/(self.max_dist_to_wall-self.min_dist_to_wall)
        return np.clip(score,0,1)
    
    def calculate_connection_score(self,r1:RobotClass, r2:RobotClass):
        dist_score = self.calculate_distance_score(r1,r2)
        sight_score = self.calculate_sight_score(r1,r2)
        return dist_score*sight_score
    


    '''
    ****************************************************************************
    * Derivative calculations
    ****************************************************************************
    '''

    def Get_gradient_vector(self):
        gradient_vector = np.zeros((self.n_nodes*2,1))
        counter = 0
        for axis in ["x","y"]:
            for r_index in range(self.n_nodes):
                grad_param = self.get_lambda2_derivative_with_respect_to(axis,r_index)
                gradient_vector[counter,0] = grad_param
                counter += 1
        return gradient_vector

    def get_lambda2_derivative_with_respect_to(self,axis,index):
        lambda_2, v_2 = self.matrix_handler.Get_second_eingenvalue_and_eingenvector()
        dL_dpi = self.derivative_laplacian_matrix_with_respect_to(axis, index)
        return v_2.T @ dL_dpi @ v_2

    def derivative_laplacian_matrix_with_respect_to(self,axis:str,index:int)->np.array:
        adjacency_matrix = self.derivate_adjacency_matrix_with_respect_to(axis,index)
        degree_vector = np.sum(adjacency_matrix, axis=1)
        degree_matrix = np.diag(degree_vector)
        return degree_matrix - adjacency_matrix

    def derivate_adjacency_matrix_with_respect_to(self,axis:str,index:int)->np.array:
        new_derivative_adjacency_matrix = np.zeros((self.n_nodes,self.n_nodes))
        for i in range(self.n_nodes):
            if i == index:
                continue
            score_derivative= self.compute_score_derivative(index,i,axis)
            new_derivative_adjacency_matrix[index,i] = score_derivative
            new_derivative_adjacency_matrix[i,index] = score_derivative
        return new_derivative_adjacency_matrix
    
    def compute_score_derivative(self,i,j,reference):
        robots_dict = [self.parent.robots_instances[robot_name] 
                       for robot_name in self.parent.robots_list]
        r1:RobotClass = robots_dict[i]
        r2:RobotClass = robots_dict[j]
        line:GridLine = self.map_handler.Get_line_between_robots(r1,r2)
        _, cell = self.map_handler.get_line_min_dist_to_obstacle(line)
        return (
            self.compute_sight_score_derivative(r1,r2,reference,cell) * 
            self.calculate_distance_score(r1,r2) +
            self.compute_distance_score_derivate(r1,r2,reference) * 
            self.calculate_sight_score(r1,r2)
        )

    def compute_sight_score_derivative(self,r1:RobotClass,r2:RobotClass,reference,grid_coord:Cell):
        if reference == "x":
            map_gradient = self.map_handler.get_gradient_x(grid_coord)
        elif reference == "y":
            map_gradient = self.map_handler.get_gradient_y(grid_coord)
        return self.get_lever_arm(r1,r2,grid_coord)*map_gradient*1/(self.max_dist_to_wall-self.min_dist_to_wall)
    
    def get_lever_arm(self, r1:RobotClass, r2:RobotClass, grid_cell:Cell): 
        full_dist = r1.Get_distance_to(r2)
        if full_dist < 1e-6:
            return 0.0
        world_point = self.map_handler.grid_to_world(grid_cell)
        obst_to_p2 = np.hypot(world_point.x - r1.pose.position.x, world_point.y - r1.pose.position.y)
        return obst_to_p2 / full_dist
    
    def compute_distance_score_derivate(self,r1:RobotClass,r2:RobotClass,reference): #derivando com respeito a p1
        dist = r1.Get_distance_to(r2)
        if reference == "x":
            return (r1.pose.position.x-r2.pose.position.x)/dist * (-1/self.max_robots_dist)
        elif reference == "y":
            return (r1.pose.position.y-r2.pose.position.y)/dist * (-1/self.max_robots_dist)