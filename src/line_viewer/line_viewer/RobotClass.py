import numpy as np
from geometry_msgs.msg import Pose,Point,Quaternion
from tf_transformations import euler_from_quaternion


class RobotClass():
    def __init__(self,name):
        self.name = name
        self.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        self.wheel_radius = None
        self.wheel_base = None
        self.yaw = 0

    def Set_pose(self,pose:Pose):
        self.pose = pose
        q = pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

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
    

    def Linear_velocity_to_xy(self,linear_velocity,angular_velocity,l):

        local_tangent_vector = angular_velocity*l

        sin_yaw = np.sin(self.yaw)
        cos_yaw = np.cos(self.yaw)

        vx = linear_velocity*cos_yaw - local_tangent_vector*sin_yaw
        vy = linear_velocity*sin_yaw + local_tangent_vector*cos_yaw

        return vx,vy
    

    def feedback_linearization_global_velocities_to_vw(self,vx,vy,l):
        j_inv = np.array([
            [np.cos(self.yaw),np.sin(self.yaw)],
            [-np.sin(self.yaw)/l,np.cos(self.yaw)/l]
        ])
        velocities_vector = np.array([[vx],
                                    [vy]])
        robot_vels = j_inv @ velocities_vector
        v_raw = robot_vels[0, 0]
        w_raw = robot_vels[1, 0]
        max_v = 0.5
        max_w = 1.5
        
        v = np.clip(v_raw, -max_v, max_v)
        w = np.clip(w_raw, -max_w, max_w)
        return v_raw,w_raw


        