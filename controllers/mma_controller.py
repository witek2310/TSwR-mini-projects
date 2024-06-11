import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class MMAController(Controller):
    def __init__(self, Tp, Kd, Kp):
        self.Kd = Kd
        self.Kp = Kp
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        m1 = ManiuplatorModel(Tp, 0.1, 0.05)
        m2 = ManiuplatorModel(Tp, 0.01, 0.01)
        m3 = ManiuplatorModel(Tp, 1.0, 0.3)
        self.models = [m1, m2, m3]
        self.i = 0
        self.prev_x = [0,0,0,0]
        self.prev_u = np.zeros((2,1))
    def choose_model(self, x):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        #predict responses of models\\
        e_min = np.inf
        for i, model in enumerate(self.models):
            x_ddot = np.linalg.inv(model.M(self.prev_x)) @ (np.array(self.prev_u).reshape((2,1)) - model.C(self.prev_x) @ np.array(self.prev_x[2:]).reshape(2,1)) 
            x_dot_new = np.array(self.prev_x[2:]).reshape((2,1)) + x_ddot * model.Tp
            x_new = np.array(self.prev_x[:2]).reshape((2,1)) + x_dot_new * model.Tp
            x_new = np.concatenate((x_new, x_dot_new), axis= 0)

            e = np.array([x]).reshape((4,1)) - x_new
            e = e.transpose() @ e
            if e < e_min:
                e_min = e
                self.i = i

        pass

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        q = np.array([x[:2]]).reshape((2,1))
        q_dot = np.array([x[2:]]).reshape((2,1))
        v = np.array([q_r_ddot]).reshape((2,1)) + self.Kd * (np.array([q_r_dot]).reshape((2,1)) - q_dot) + self.Kp * ( np.array([q_r]).reshape(2,1) - q) # TODO: add feedback
        print(self.i)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v+ C @ q_dot
        self.prev_u = u
        self.prev_x = x
        return u
