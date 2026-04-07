
import numpy as np
from .RobotClass import RobotClass
from typing import TYPE_CHECKING, List
import copy
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

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
        self.degree_matrix = self.generate_degree_matrix(self.adjacency_matrix)
        self.laplacian_matrix = self.degree_matrix - self.adjacency_matrix

    def generate_degree_matrix(self,adjacency_matrix):
        degree_vector = np.sum(adjacency_matrix, axis=1)
        return np.diag(degree_vector)

    def Get_second_eingenvalue_and_eingenvector(self):
        eigenvalues, eigenvectors = np.linalg.eigh(self.laplacian_matrix)
        lambda_2 = eigenvalues[1]
        v_2 = eigenvectors[:, 1]
        return lambda_2, v_2
    
    def Set_laplacian_matrix(self,matrix):
        self.laplacian_matrix = matrix
        diag_values = np.diag(self.laplacian_matrix)
        self.degree_matrix = np.diag(diag_values)
        self.adjacency_matrix = self.degree_matrix - self.laplacian_matrix

    def Get_laplacian_matrix(self):
        return self.laplacian_matrix


class MathHandler:
    def __init__(self,
        parent:'RobotsMathNode',
        distance_score_scale,
        distance_score_offset,
        sight_score_scale,
        sight_score_offset
    ):
        self.n_nodes = len(parent.robots_list)
        self.matrix_handler = MatrixHandler(self.n_nodes)
        self.map_handler = parent.map_handler
        self.parent = parent
        self.distance_score_scale = distance_score_scale
        self.distance_score_offset = distance_score_offset
        self.sight_score_scale = sight_score_scale
        self.sight_score_offset = sight_score_offset

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
                score = self.calculate_robots_connection_score(r1,r2)
                self.Update_laplacian_matrix(score,i,j)

    def calculate_distance_score(self,p1:Point, p2:Point):
        distance = np.hypot(p1.x-p2.x, p1.y-p2.y)
        return 1/(1+np.exp(distance-6))
    
    def calculate_robots_sight_score(self,r1:RobotClass, r2:RobotClass):
        return self.calculate_positions_sight_score(r1.pose.position,r2.pose.position)

    def calculate_positions_sight_score(self, p1, p2):
        line = self.map_handler.Get_line_between_positions(p1, p2)
        if line is None: return 1e-6 

        min_obstacle_dist, min_cell = self.map_handler.get_line_min_dist_to_obstacle(line)
        score_sdf = 1.0 / (1.0 + np.exp(-6 * (min_obstacle_dist - 0.5)))

        
        return score_sdf
    


    def generate_sigmoid_score_chart(self,title,xlabel,ylabel,scale,offset,path):
        sight_equation = lambda x: 1 / (1 + np.exp(scale * (x - offset)))

        visual_span = 5.0 / abs(scale)

        x_min = offset - visual_span
        x_max = offset + visual_span

        x = np.linspace(x_min, x_max, 500)
        y = sight_equation(x)

        fig, ax = plt.subplots(figsize=(16, 10))

        ax.plot(x, y, linewidth=4, color='#007ACC')

        ax.set_title(title, pad=20, fontweight='bold', color='#333333', size=20)
        ax.set_xlabel(xlabel, labelpad=15, size=15)
        ax.set_ylabel(ylabel, labelpad=15, size=15)

        equation_text = rf'$f(x) = \frac{{1}}{{1 + e^{{{scale}(x - {offset})}}}}$'
        
        if (scale > 0):
            box_ypos = 0.1
        else:
            box_ypos = 0.9

        ax.text(0.05, box_ypos, equation_text, transform=ax.transAxes, fontsize=22,
                bbox=dict(facecolor='white', alpha=0.8, edgecolor='#cccccc', boxstyle='round,pad=0.5'))

        ax.grid(True, which='major', linestyle='--', alpha=0.6)
        ax.minorticks_on()
        ax.grid(True, which='minor', linestyle=':', alpha=0.3)
        
        ax.set_xlim([x_min - 0.05 * (x_max - x_min), x_max])
        ax.set_ylim([-0.05, 1.05])

        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        fig.savefig(path, dpi=300, bbox_inches='tight')

        plt.close(fig)


    def generate_sight_score_chart(self, path):
        return self.generate_sigmoid_score_chart(
            'Sight Score Calculation Curve',
            'Distance Line x Obstacle [m]',
            'Sight Score',
            self.sight_score_scale,
            self.sight_score_offset,
            path
        )

    def generate_distance_score_chart(self, path):
        return self.generate_sigmoid_score_chart(
            'Distance Score Calculation Curve',
            'Distance between robots [m]',
            'Distance Score',
            self.distance_score_scale,
            self.distance_score_offset,
            path
        )








    def calculate_positions_connection_score(self,p1:Point, p2:Point):
        dist_score = self.calculate_distance_score(p1,p2)
        sight_score = self.calculate_positions_sight_score(p1,p2)
        return dist_score *sight_score
    

    def calculate_robots_connection_score(self,r1:RobotClass, r2:RobotClass):
        p1 = r1.pose.position
        p2 = r2.pose.position
        return self.calculate_positions_connection_score(p1,p2)

    



    '''
    ****************************************************************************
    * Derivative calculations
    ****************************************************************************
    '''



    def Get_gradient_vector_numeric_way(self,dt):
        gradient_vector = np.zeros((self.n_nodes*2,1))
        counter = 0
        for r_index in range(self.n_nodes):
            for axis in ["x","y"]:
                grad_param = self.Generate_numeric_Laplacian_derivative(axis,r_index,dt)
                gradient_vector[counter,0] = grad_param
                counter += 1
        return gradient_vector


    def Generate_numeric_Laplacian_derivative(self,axis:str,index:int,dt):
        new_adjacency_matrix = copy.deepcopy(self.matrix_handler.adjacency_matrix)
        robots_dict = [self.parent.robots_instances[robot_name] 
                       for robot_name in self.parent.robots_list]
        for i in range(self.n_nodes):
            if i == index:
                continue
            r1:RobotClass = robots_dict[index]
            r2:RobotClass = robots_dict[i]
            p1 = copy.copy(r1.pose.position)
            p2 = copy.copy(r2.pose.position)
            if axis == "x":
                p1.x += dt
            elif axis == "y":
                p1.y += dt
            else:
                raise Exception
            score = self.calculate_positions_connection_score(p1,p2)
            new_adjacency_matrix[index,i] = score
            new_adjacency_matrix[i,index] = score
        degree_matrix = self.matrix_handler.generate_degree_matrix(new_adjacency_matrix)
        new_laplacian_matrix = degree_matrix - new_adjacency_matrix
        eigenvalues, eigenvectors = np.linalg.eigh(new_laplacian_matrix)
        new_lambda_2 = eigenvalues[1]
        old_lambda_2,_ = self.matrix_handler.Get_second_eingenvalue_and_eingenvector()
        return (new_lambda_2-old_lambda_2)/dt
