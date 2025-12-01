from geometry_msgs.msg import Point
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib.pyplot as plt


from .RobotClass import RobotClass

class Cell:
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def Get_distance_to(self,other_cell:'Cell') -> float:
        return np.hypot(self.x-other_cell.x, self.y-other_cell.y)

class GridLine():
    def __init__(self,c1:Cell,c2:Cell):
        self.c1 = c1
        self.c2 = c2
        self.x_values = None
        self.y_values = None
        self.Create_line()

    def Create_line(self):
        dist_pixels = self.c1.Get_distance_to(self.c2)
        num_points = max(int(dist_pixels), 1)
        self.x_values = np.linspace(self.c1.x, self.c2.x, num_points).astype(int)
        self.y_values = np.linspace(self.c1.y, self.c2.y, num_points).astype(int)

    def Get_distances_to_wall(self,dist_transform):
        return dist_transform[self.y_values, self.x_values]
    
    def Get_cell_from_index(self,index):
        return Cell(self.x_values[index],self.y_values[index])

class MapHandler:
    def __init__(self,map_info,map_data):
        self.map_info = map_info
        self.map_data = map_data
        self.dist_transform = None
        self.dist_transform_x_derivative = None
        self.dist_transform_y_derivative = None

    def grid_to_world(self, cell:Cell):
        world_point = Point()
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        world_point.x = (cell.x * res) + origin_x + (0.5 * res)
        world_point.y = (cell.y * res) + origin_y + (0.5 * res)
        world_point.z = 0.0
        return world_point

    def world_to_grid(self, world_point):
        if self.map_info is None:
            return None
            
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        width = self.map_info.width
        height = self.map_info.height

        map_x = int((world_point.x - origin_x) / resolution)
        map_y = int((world_point.y - origin_y) / resolution)

        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return None
            
        return Cell(map_x, map_y)


    def compute_map_distance(self):
        width = self.map_info.width
        height = self.map_info.height
        grid = np.array(self.map_data).reshape((height, width))
        mask_free = np.where((grid > 90) | (grid < 0), 0, 1)
        dist_to_obstacle = distance_transform_edt(mask_free)
        mask_obstacle = 1 - mask_free
        dist_to_free = distance_transform_edt(mask_obstacle)
        self.dist_transform = (dist_to_obstacle - dist_to_free) * self.map_info.resolution
        dy, dx = np.gradient(self.dist_transform, self.map_info.resolution)
        self.dist_transform_y_derivative = dy
        self.dist_transform_x_derivative = dx

    def generate_distances_colormap(self,path):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.dist_transform, cmap='jet', origin='lower') 
        plt.colorbar(label='Distância até a parede (m)')
        plt.title('Transformada de Distância Euclidiana (EDT)')
        plt.savefig(path)
        plt.close()

    def generate_gradient_colormap(self,path,axis):
        plt.figure(figsize=(10, 10))
        if axis =="x":
            plt.imshow(self.dist_transform_x_derivative, cmap='jet', origin='lower') 
        elif axis =="y":
            plt.imshow(self.dist_transform_y_derivative, cmap='jet', origin='lower') 
        else:
            return
        plt.colorbar(label='Distância até a parede (m)')
        plt.title('Transformada de Distância Euclidiana (EDT)')
        plt.savefig(path)
        plt.close()


    def Get_line_between_robots(self, r1: RobotClass, r2: RobotClass):
        return self.Get_line_between_positions(r1.pose.position,r2.pose.position)

    

    def Get_line_between_positions(self, p1, p2):
        cell_1 = self.world_to_grid(p1)
        cell_2 = self.world_to_grid(p2)
        if cell_1 is None or cell_2 is None:
            return None
            
        return GridLine(cell_1, cell_2)



    def get_line_min_dist_to_obstacle(self, grid_line:GridLine):
        wall_distances = grid_line.Get_distances_to_wall(self.dist_transform)
        min_idx = np.argmin(wall_distances)
        min_dist = wall_distances[min_idx]
        cell_min_point = grid_line.Get_cell_from_index(min_idx)
        return min_dist, cell_min_point
    

    def get_gradient_x(self, cell:Cell):
        return self.dist_transform_x_derivative[cell.y, cell.x]

    def get_gradient_y(self, cell:Cell):
        return self.dist_transform_y_derivative[cell.y, cell.x]