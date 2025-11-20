from geometry_msgs.msg import Point
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib.pyplot as plt

class MapHandler:
    def __init__(self,map_info,map_data):
        self.map_info = map_info
        self.map_data = map_data


    def grid_to_world(self, grid_x, grid_y):
        world_point = Point()
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        world_point.x = (grid_x * res) + origin_x + (0.5 * res)
        world_point.y = (grid_y * res) + origin_y + (0.5 * res)
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
            
        return (map_x, map_y)



    def compute_map_distance(self):
        width = self.map_info.width
        height = self.map_info.height
        grid = np.array(self.map_data).reshape((height, width))
        binary_map = np.where((grid > 90) | (grid < 0), 0, 1)
        self.dist_transform = distance_transform_edt(binary_map) * self.map_info.resolution

    def generate_distances_colormap(self,path):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.dist_transform, cmap='jet', origin='lower') 
        
        plt.colorbar(label='Distância até a parede (m)')
        plt.title('Transformada de Distância Euclidiana (EDT)')
        
        plt.savefig(path)
        plt.close()
        


    def get_line_min_dist_to_obstacle(self, p1, p2):
        grid_x0, grid_y0 = self.world_to_grid(p1)
        grid_x1, grid_y1 = self.world_to_grid(p2)

        dist_pixels = np.hypot(grid_x1-grid_x0, grid_y1-grid_y0)
        num_points = int(dist_pixels)
        
        if num_points == 0: 
            val = self.dist_transform[grid_y0, grid_x0]
            return val, p1

        x_vals = np.linspace(grid_x0, grid_x1, num_points).astype(int)
        y_vals = np.linspace(grid_y0, grid_y1, num_points).astype(int)

        distances_along_line = self.dist_transform[y_vals, x_vals]

        min_dist = np.min(distances_along_line)
        return min_dist