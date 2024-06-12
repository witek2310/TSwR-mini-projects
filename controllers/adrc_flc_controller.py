import numpy as np

# from models.free_model import FreeModel
from observers.eso import ESO
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
from models.manipulator_model import ManiuplatorModel
# from models.ideal_model import IdealModel
from .pd_controller import PDDecentralizedController

class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.model = ManiuplatorModel(Tp, 0.01, 0.04)
        self.Kp = Kd
        self.Kd = Kd
        self.L = self.calculate_ls(p)
        M = self.model.M((0,0,0,0))
        C = self.model.C((0,0,0,0))
        temp = -np.linalg.inv(M)@C
        W = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0]])
        
        A = np.array([[0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, temp[0,0], temp[0,1], 1, 0],
                      [0,0,temp[1,0], temp[1,1], 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])
        
        M_inv = np.linalg.inv(M)
        B = np.array([[0, 0],
                      [0, 0],
                      [M_inv[0,0], M_inv[0,1]],
                      [M_inv[1,0], M_inv[1,1]],
                      [0, 0],
                      [0, 0]])
        
        self.eso = ESO(A, B, W, self.L, q0, Tp)
        self.update_params(q0)
        self.prev_u = np.zeros((2,1))

    def update_params(self, x):
        ### TODO Implement procedure to set eso.A and eso.B
        M = self.model.M(x)
        C = self.model.C(x)
        M_inv = np.linalg.inv(M)
        temp = -M_inv@C

        self.eso.A = np.array([[0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, temp[0,0], temp[0,1], 1, 0],
                      [0,0,temp[1,0], temp[1,1], 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])

        self.eso.B = np.array([[0, 0],
                      [0, 0],
                      [M_inv[0,0], M_inv[0,1]],
                      [M_inv[1,0], M_inv[1,1]],
                      [0, 0],
                      [0, 0]])
    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement centralized ADRFLC
        self.update_params(x)
        self.eso.update(x[:2], [self.prev_u])
        state = self.eso.get_state()[0][0]

        q_hat = state[:2, 0].reshape((2,1))
        q_dot_hat = state[2:4, 0].reshape((2,1))
        f = state[4:, 0].reshape((2,1))

        
        u_pd = self.Kd @ (np.array([q_d_dot]).reshape((2,1)) - q_dot_hat) + self.Kp @ (np.array([q_d]).reshape((2,1)) - q_hat)

        v = u_pd + np.array([q_d_ddot]).reshape((2,1))

        u = self.model.M(x) @ (v - f) + self.model.C(x) @ q_dot_hat
        
        self.prev_u = u

        return u

    def calculate_ls(self, p):
        return np.array([[3*p[0], 0],
                         [0, 3* p[1]],
                         [3 * p[0] ** 2, 0],
                         [0, 3 * p[0] ** 2],
                         [p[0]**3, 0],
                         [0, p[1]**3]])