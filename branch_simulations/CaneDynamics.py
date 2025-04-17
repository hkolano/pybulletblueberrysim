# import numpy as np

class CaneDynamics:
    def __init__(self, njoints):
        self.njoints = njoints
        self.Kps = [0]*njoints 
        self.Kds = [0]*njoints

    def reset_values(self):
        self.Kps = [1.5]*self.njoints 
        self.Kds = [.1]*self.njoints

    def set_to_uniform_values(self, Kp, Kd):
        self.Kps = [Kp]*self.njoints
        self.Kds = [Kd]*self.njoints

    def set_linearly_increasing(self, Kp_low, Kp_high, Kd_low, Kd_high):
        self.Kps = np.linspace(Kp_low, Kp_high, num=self.njoints)
        self.Kds = np.linspace(Kd_low, Kd_high, num=self.njoints)

    def set_linearly_decreasing(self, Kp_high, Kp_low, Kd_high, Kd_low):
        self.Kps = np.linspace(Kp_high, Kp_low, num=self.njoints)
        self.Kds = np.linspace(Kd_high, Kd_low, num=self.njoints)




