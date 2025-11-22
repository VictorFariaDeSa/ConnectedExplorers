
import numpy as np
from .RobotClass import RobotClass
from .MapHandler import MapHandler

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

    def Get_second_eingenvalue_and_eingenvector(self,laplacian_matrix):
        eigenvalues, eigenvectors = np.linalg.eigh(laplacian_matrix)
        lambda_2 = eigenvalues[1]
        v_2 = eigenvectors[:, 1]
        return lambda_2, v_2
    
    def Get_laplacian_matrix(self):
        return self.laplacian_matrix


class MathHandler:
    def __init__(self,
        map_handler:MapHandler,
        max_robot_dist:int,
        n_nodes:int,
        min_dist_to_wall:int,
        max_dist_to_wall:int
    ):
        self.matrix_handler = MatrixHandler(n_nodes)
        self.max_robot_dist = max_robot_dist
        self.map_handler = map_handler
        self.min_dist_to_wall = min_dist_to_wall
        self.max_dist_to_wall = max_dist_to_wall


    def Update_laplacian_matrix(self,score,i,j):
        return self.matrix_handler.Update_laplacian_matrix(score,i,j)
    
    def get_second_eingenvalue_and_eingenvector(self,laplacian_matrix):
        return self.matrix_handler.get_second_eingenvalue_and_eingenvector(self,laplacian_matrix)

    def Get_laplacian_matrix(self):
        return self.matrix_handler.Get_laplacian_matrix()

    def refresh_laplacian_matrix(self,robots_list):
        for i, r1 in enumerate(robots_list):
            for j, r2 in enumerate(robots_list):
                if i >= j:continue
                score = self.calculate_connection_score(r1,r2)
                self.Update_laplacian_matrix(score,i,j)


    def calculate_distance_score(self,r1:RobotClass, r2:RobotClass):
        distance = r1.Get_distance_to(r2)
        return np.clip((self.max_robot_dist - distance)/(self.max_robot_dist),0,1)
    
    def calculate_sight_score(self,r1:RobotClass, r2:RobotClass):
        line = self.map_handler.Get_line_between_robots(r1,r2)
        min_obstacle_dist,_ = self.map_handler.get_line_min_dist_to_obstacle(line)
        score = (min_obstacle_dist-self.min_dist_to_wall)/(self.max_dist_to_wall-self.min_dist_to_wall)
        return np.clip(score,0,1)
    
    def calculate_connection_score(self,r1:RobotClass, r2:RobotClass):
        dist_score = self.calculate_distance_score(r1,r2)
        sight_score = self.calculate_sight_score(r1,r2)
        return dist_score*sight_score