import numpy as np
from geometry_msgs.msg import Pose,Point,Quaternion


class RobotClass():
    def __init__(self,name):
        self.name = name
        self.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        self.wheel_radius = None
        self.wheel_base = None

    def Set_pose(self,pose):
        self.pose = pose

    def Get_distance_to(self,other_robot:'RobotClass')->float:
        p1 = self.pose.position
        p2 = other_robot.pose.position
        return np.hypot(p2.x-p1.x, p2.y-p1.y)
    
    def Get_pos_as_tuple(self):
        return (self.pose.position.x,self.pose.position.y)
    
    def Solve_inverse_kin(self,v,w):
        wr = (2*v + w*self.wheel_base)/(2*self.wheel_radius)
        wl = (2*v - w*self.wheel_base)/(2*self.wheel_radius)
        return (wl,wr)

    def Solve_direct_kin(self):
        pass
    


        