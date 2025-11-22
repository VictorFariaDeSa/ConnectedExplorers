import numpy as np
from geometry_msgs.msg import Point


class RobotClass():
    def __init__(self,name):
        self.name = name
        self.position = Point(x=0.0, y=0.0, z=0.0)

    def Set_position(self,position):
        self.position = position

    def Get_distance_to(self,other_robot:'RobotClass')->float:
        p1 = self.position
        p2 = other_robot.position
        return np.hypot(p2.x-p1.x, p2.y-p1.y)
    
    def Get_pos_as_tuple(self):
        return (self.position.x,self.position.y)