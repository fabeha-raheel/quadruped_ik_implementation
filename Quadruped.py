from math import pi, acos, asin, sqrt

class Quadruped:
    def __init__(self, link_lengths=[1, 1, 1], bend="backward"):
        # LEG PARAMS
        self.link_lengths = link_lengths    
        # self.L1 = self.link_lengths[0]  # Length of Link 1 (Hip)
        # self.L2 = self.link_lengths[1]  # Length of Link 2 (Tigh)
        # self.L3 = self.link_lengths[2]  # Length of LInk 3 (Lower Leg / Shank / Calf)
        if bend == "backward": self.FORWARD_BEND = False
        else: self.FORWARD_BEND = True    # Set True if legs are in Forward Bend condition, else False

    def set_link_lengths(self, link_lengths):
        self.link_lengths = link_lengths

    def get_link_lengths(self):
        return self.link_lengths


    def compute_single_leg_ik(self, position=[-1 , -1 , -1]):

        x, y, z = position
        assert y < 0, "Invalid y coordinate input!"

        alpha = acos(abs(z) / sqrt(y**2 + z**2))
        beta = acos(self.link_lengths[0] / sqrt(y**2 + z**2))

        if z>0: q1 = alpha - beta  
        else: q1 = pi - alpha - beta

        y_prime = -sqrt(y**2 + z**2 - self.link_lengths[1]**2)

        phi = acos(abs(x) / sqrt(x**2 + y_prime**2))
        psi = acos((self.link_lengths[1]**2 + x**2 + y_prime**2 - self.link_lengths[2]**2) / (2*self.link_lengths[1]*sqrt(x**2 + y_prime**2)))

        if not self.FORWARD_BEND:
            if x>0: q2 = (pi/2 - psi - phi) 
            else: q2 = (-pi/2 - psi + phi)
            q3 = acos((self.link_lengths[1]**2 + self.link_lengths[2]**2 - x**2 - y_prime**2) / 2*self.link_lengths[1]*self.link_lengths[2])
        else:
            if x>0: q2 = (pi/2 + psi - phi)
            else: q2 = (-pi/2 + psi + phi)
            q3 = -acos((self.link_lengths[1]**2 + self.link_lengths[2]**2 - x**2 - y_prime**2) / 2*self.link_lengths[1]*self.link_lengths[2])

        assert abs(q2) < pi/2, "q2 greater than limits"

        return (q1, q2, q3)

if __name__ == "__main__":

    # INPUTS
    position = [0 , -1 , -1]
    L1 = 0.105  # Length of Link 1 (Hip) m
    L2 = 0.225  # Length of Link 2 (Thigh) m
    L3 = 0.230  # Length of Link 3 (Lower Leg / Shin) m

    robot = Quadruped()
    robot.set_link_lengths([L1, L2, L3])

    joint_angles = robot.compute_single_leg_ik(position)

    print("Calculated Joint Angles:")
    print("Theta 1: ", joint_angles[0])
    print("Theta 2: ", joint_angles[1])
    print("Theta 3: ", joint_angles[2])