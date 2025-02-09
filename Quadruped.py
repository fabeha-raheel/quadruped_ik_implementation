from math import pi, acos, asin, sqrt, cos, sin
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Quadruped:
    def __init__(self, link_lengths=[1, 1, 1], bend="backward"):
        # LEG PARAMS
        self.link_lengths = link_lengths    
        self.FORWARD_BEND  = bend != "backward"    # Set True if legs are in Forward Bend condition, else False

    def set_link_lengths(self, link_lengths):
        self.link_lengths = link_lengths

    def get_link_lengths(self):
        return self.link_lengths


    def compute_single_leg_ik(self, position=[-1 , -1 , -1]):

        x, y, z = position
        assert y < 0, "Invalid y coordinate input!"

        alpha = acos(abs(z) / sqrt(y**2 + z**2))
        beta = acos(self.link_lengths[0] / sqrt(y**2 + z**2))

        print("alpha: ", alpha)
        print("beta: ", beta)

        if z>0: q1 = alpha - beta  
        else: q1 = pi - alpha - beta

        y_prime = -1*sqrt(y**2 + z**2 - self.link_lengths[0]**2)

        print("yprime: ", y_prime)

        phi = acos(abs(x) / sqrt(x**2 + y_prime**2))
        psi = acos((self.link_lengths[1]**2 + x**2 + y_prime**2 - self.link_lengths[2]**2) / (2*self.link_lengths[1]*sqrt(x**2 + y_prime**2)))

        print("phi: ", phi)
        print("psi: ", psi)
        
        if not self.FORWARD_BEND:
            if x>0: q2 = (pi/2 - psi - phi) 
            else: q2 = (-pi/2 - psi + phi)
            q3 = acos((self.link_lengths[1]**2 + self.link_lengths[2]**2 - x**2 - y_prime**2) / (2*self.link_lengths[1]*self.link_lengths[2]))
        else:
            if x>0: q2 = (pi/2 + psi - phi)
            else: q2 = (-pi/2 + psi + phi)
            q3 = -1*acos((self.link_lengths[1]**2 + self.link_lengths[2]**2 - x**2 - y_prime**2) / (2*self.link_lengths[1]*self.link_lengths[2]))

        assert abs(q2) < pi/2, "q2 greater than limits"

        return (q1, q2, q3)
    
    def compute_forward_kinematics(self, joint_angles):
        q1, q2, q3 = joint_angles
        L1, L2, L3 = self.link_lengths

        # Hip position
        hip = np.array([0, L1*sin(q1), L1*cos(q1)])
        hip = self.leg_to_world_transform(hip)

        # Knee position
        knee = hip + np.array([
            L1 * np.cos(q1),
            L1 * np.sin(q1),
            0
        ])
        knee = self.leg_to_world_transform(knee)

        # Foot position
        foot = knee + np.array([
            L2 * np.cos(q1 + q2),
            L2 * np.sin(q1 + q2),
            0
        ])

        # Final foot position with q3
        foot_final = foot + np.array([
            L3 * np.cos(q1 + q2 + q3),
            L3 * np.sin(q1 + q2 + q3),
            0
        ])
        foot_final = self.leg_to_world_transform(foot_final)

        return np.array([hip, knee, foot, foot_final])
    
    def leg_to_world_transform(self, positions_leg):
        """
        Converts positions from leg frame to world frame.
        
        positions_leg: Nx3 array of (x_leg, y_leg, z_leg) coordinates
        Returns: Nx3 array of transformed (x_world, y_world, z_world) coordinates
        """
        R_y_90 = np.array([
            [0,  0, 1],
            [0,  1, 0],
            [-1, 0, 0]
        ])

        positions_world = np.dot(positions_leg, R_y_90.T)  # Apply transformation
        return positions_world

    def plot_leg(self, positions):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        x_vals = positions[:, 0]
        y_vals = positions[:, 1]
        z_vals = positions[:, 2]

        ax.plot(x_vals, y_vals, z_vals, 'o-', markersize=8, label="Leg Links")

        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis (Vertical)")
        ax.set_zlabel("Z Axis (Hip Rotation Axis)")
        ax.legend()
        ax.set_title("Quadruped Leg Visualization")

        ax.set_xlim([-0.3, 0.3])
        ax.set_ylim([-0.3, 0.3])
        ax.set_zlim([-0.3, 0.3])

        plt.show()

if __name__ == "__main__":

    # INPUTS
    # position = [0.03 , -0.06 , -0.23]
    position = [-0.05 , -0.324285661 , 0.17]
    L1 = 0.105  # Length of Link 1 (Hip) m
    L2 = 0.225  # Length of Link 2 (Thigh) m
    L3 = 0.230  # Length of Link 3 (Lower Leg / Shin) m

    robot = Quadruped()
    robot.set_link_lengths([L1, L2, L3])

    joint_angles = robot.compute_single_leg_ik(position)

    print("Calculated Joint Angles:")
    print("Theta 1: ", math.degrees(joint_angles[0]))
    print("Theta 2: ", math.degrees(joint_angles[1]))
    print("Theta 3: ", math.degrees(joint_angles[2]))

    # positions = robot.compute_forward_kinematics(joint_angles)

    # # Plot the leg in 3D
    # robot.plot_leg(positions)