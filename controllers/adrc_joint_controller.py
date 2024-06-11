import numpy as np
from observers.eso import ESO
from .controller import Controller
from .pd_controller import PDDecentralizedController

class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd

        A = np.array([[0, 1, 0], 
                      [0, 0, 1],
                      [0, 0, 0]])
        B = np.array([[0],
                      [self.b], 
                      [0]])
        L = self.calculate_ls(p)
        W = np.array([[1, 0, 0]])
        self.eso = ESO(A, B, W, L, q0, Tp)
        self.prev_u = 0
        self.loc_control = PDDecentralizedController(3, 1)

    def set_b(self, b):
        ### TODO update self.b and B in ESO
        self.eso.B = np.array([[0], [self.b], [0]])
        pass

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement ADRC

        self.eso.update([x[0]], [self.prev_u])
        q_hat, q_dot_hat, f = self.eso.get_state()
        


        u_pd = self.loc_control.calculate_control(x[0], q_dot_hat, q_d,q_d_dot, q_d_ddot)
        v = u_pd + q_d_ddot

        u = (v -f )/ self.b
        
        
        self.prev_u = u
        return u
    
    # calculates l1, l2, l3 based on p
    # p must be bigger than 0 otherwise the obserwer will be unstable
    def calculate_ls(self, p):
        return np.array([[3*p], [3 * p ** 2], [p**3]])