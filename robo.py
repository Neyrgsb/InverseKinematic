import numpy as np

class joint ():
    def __init__(self, position, lenght, theta, psi):
        self.position = position
        self.rotation_xy = theta
        self.rotation_xz = psi
        self.lenght = lenght

    def set_pos(self, position):
        self.position = position
    
    def get_end_pos(self):
        x = self.position[0] + self.lenght * np.cos(self.rotation_xy) * np.cos(self.rotation_xz)
        y = self.position[1] + self.lenght * np.sin(self.rotation_xy)
        z = self.position[2] + self.lenght * np.cos(self.rotation_xy) * np.sin(self.rotation_xz)
        return (x, y, z)

class Robot():
    def __init__(self, position , arm_lenght, arm_orientation_xy, arm_orientation_xz, wrist_lenght, wrist_orientation_xy):
        self.arm = joint(position, arm_lenght, arm_orientation_xy, arm_orientation_xz)
        self.wrist = joint(self.arm.get_end_pos(), wrist_lenght, wrist_orientation_xy, arm_orientation_xz)
        
    def fx(self, theta, phi, psi):
        # compute y final position,  arm angle = theta, wrist angle = phi
        return ((self.arm.lenght * np.cos(theta)) + (self.wrist.lenght * np.cos(theta + phi))) * np.cos(psi)

    def fy(self, theta, phi, psi):
        # compute y final position,  arm angle = theta, wrist angle = phi
        return (self.arm.lenght * np.sin(theta)) + (self.wrist.lenght * np.sin(theta + phi))

    def fz(self, theta, phi, psi):
        # compute x final positioon, arm angle = theta, wrist angle = phi
        return ((self.arm.lenght * np.cos(theta)) + (self.wrist.lenght * np.cos(theta + phi))) * np.sin(psi)  


    def rotate(self, theta, phi, psi):
        self.arm.rotation_xy = theta
        self.arm.rotation_xz = psi
        self.wrist = joint(self.arm.get_end_pos(), self.wrist.lenght, theta+phi, psi)
        


    def Jacobian(self):

        theta = self.arm.rotation_xy
        phi = self.wrist.rotation_xy
        psi = self.arm.rotation_xz

        J = np.ones((3,3))
        
        # derivadas parciais de fx
        J[0,0] = -1 * ((self.arm.lenght * np.sin(theta)) + self.wrist.lenght * np.sin(theta + phi)) * np.cos(psi) 
        J[0,1] = -1 * self.wrist.lenght * (np.sin(theta + phi)) * np.cos(psi)
        J[0,2] = -1 * ((self.arm.lenght * np.cos(theta)) + (self.wrist.lenght * np.cos(theta + phi))) * np.sin(psi)

        # derivadas parciais de fy
        J[1,0] =  self.arm.lenght * np.cos(theta) + self.wrist.lenght * np.cos(theta + phi)
        J[1,1] = self.wrist.lenght * np.cos(theta + phi) 
        J[1,2] = 0

        # derivadas parciais de fz
        J[2,0] = -1 * ((self.arm.lenght * np.sin(theta)) + self.wrist.lenght * np.sin(theta + phi)) * np.sin(psi) 
        J[2,1] = -1 * self.wrist.lenght * (np.sin(theta + phi)) * np.sin(psi)
        J[2,2] = ((self.arm.lenght * np.cos(theta)) + (self.wrist.lenght * np.cos(theta + phi))) * np.cos(psi)
        """
        # derivadas parciais de fx
        J[0,0] = -1 * (self.arm.lenght * np.sin(theta)) * np.cos(psi) 
        J[0,1] = -1 * self.wrist.lenght * (np.sin(phi)) * np.cos(psi)
        J[0,1] = -1 * ((self.arm.lenght * np.cos(theta)) + (self.wrist.lenght * np.cos(phi))) * np.sin(psi)

        # derivadas parciais de fy
        J[1,0] =  self.arm.lenght * np.cos(theta) 
        J[1,1] = self.wrist.lenght * np.cos(phi) 
        J[1,2] = 0

        # derivadas parciais de fz
        J[2,0] = -1 * ((self.arm.lenght * np.sin(theta))) * np.sin(psi) 
        J[2,1] = -1 * self.wrist.lenght * (np.sin(phi)) * np.sin(psi)
        J[2,2] = ((self.arm.lenght * np.cos(theta)) + (self.wrist.lenght * np.cos(theta + phi))) * np.cos(psi)
        """
        return J

