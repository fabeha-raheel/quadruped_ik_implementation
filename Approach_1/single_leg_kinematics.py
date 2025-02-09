
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt
from util import RotMatrix3D, point_to_rad

class single_leg_kinematics():
    
    def __init__(self):
        
        # note: leg IDs
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        self.L1 = 0.045
        self.L2 = 0.1115
        self.L3 = 0.155
        self.phi = radians(90)

        self.single_leg_origin = [0, 0, 0]


    # Single Leg IK calculator
    def single_leg_IK_calc(self, xyz, rot, is_right=False): 

        x, y, z = xyz[0], xyz[1], xyz[2]    # unpack coordinates
        
        # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = norm([0,y,z])   
        
        # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        # a_2 : angle bewtween len_A and leg's projection line on YZ plane
        # a_3 : angle between link1 and length len_A
        a_1 = point_to_rad(y,z)                     
        a_2 = asin(sin(self.phi)*self.L1/len_A) 
        a_3 = pi - a_2 - self.phi                   
        
        # angle of link1 about the x-axis 
        if is_right: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*pi: theta_1 -= 2*pi
        
        j2 = array([0,self.L1*cos(theta_1),self.L1*sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2 # vector from j2 to j4
        
        if is_right: R = theta_1 - self.phi - pi/2
        else: R = theta_1 + self.phi - pi/2
        
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        
        # xyz in the rotated coordinate system + offset due to L1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        len_B = norm([x_, z_]) # norm(j4-j2)
        
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.L2 + self.L3): 
            len_B = (self.L2 + self.L3) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and L2
        # b_3 : angle between L2 and L3
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.L2**2 + len_B**2 - self.L3**2) / (2 * self.L2 * len_B)) 
        b_3 = acos((self.L2**2 + self.L3**2 - len_B**2) / (2 * self.L2 * self.L3))  
        
        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3
        
        # CALCULATE THE COORDINATES OF THE JOINTS FOR VISUALIZATION
        j1 = np.array([0,0,0])
        
        # calculate joint 3
        j3_ = np.reshape(np.array([self.L2*cos(theta_2),0, self.L2*sin(theta_2)]),[3,1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j3_, [1,3])).flatten()
        
        # calculate joint 4
        j4_ = j3_ + np.reshape(np.array([self.L3*cos(theta_2+theta_3),0, self.L3*sin(theta_2+theta_3)]), [3,1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j4_, [1,3])).flatten()
        
        # modify angles to match robot's configuration (i.e., adding offsets)
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        # print(degrees(angles[0]))
        return [angles[0], angles[1], angles[2], j1, j2, j3, j4]
       
    # get coordinates of leg joints relative to j1
    def leg_pose(self, xyz, rot, legID, is_radians, center_offset=[0,0,0]):

        # check is the leg is from the right side 
        is_right = (legID in self.right_legs)
        
        # get the coordinates of each joints relative to the leg's origin
        pose_relative = self.single_leg_IK_calc(xyz, is_right)[3:]
        
        pose_true = array(pose_relative)
        return pose_true.transpose()
       
    # plot leg 
    def plot_leg(self, ax, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = self.leg_pose(xyz, rot, legID, is_radians, center_offset)
        
        # plot coordinates
        # ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'b')

        colors = ['b', 'r', 'g']  # blue, red, green

        # Plot each link with the corresponding color
        for i in range(len(colors)):
            ax.plot3D(
                asarray(p[0, i:i+2]).flatten(), 
                asarray(p[1, i:i+2]).flatten(), 
                asarray(p[2, i:i+2]).flatten(), 
                colors[i]
            )

        return
    
    # plot single leg
    def plot_single_leg(self, xyz, rot=[0,0,0], leg_id=2, is_radians=True, limit=0.250, center_offset=[0,0,0]):
    
        ax = self.ax_view(limit)  # set the view

        self.plot_leg(ax,xyz,rot,leg_id, is_radians, center_offset) 
        
        # show figure
        plt.show()
        return
        
    # TO-DO : modify this function depending on your robot's configuration
    # adjusting angle for specific configurations of motors, incl. orientation
    # this will vary for each robot (possibly for each leg as well)
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi; # add offset 
        
        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 45*pi/180 # 45 degrees initial offset
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi
            else: theta_1 = angles[0]
            
            theta_2 = -angles[1] - 45*pi/180
        
        theta_3 = -angles[2] + 45*pi/180
        return [theta_1, theta_2, theta_3]
        
    # set view  
    @staticmethod
    def ax_view(limit):
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax