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
        self.loc_control = PDDecentralizedController(kp, kd)

    def set_b(self, b):
        ### TODO update self.b and B in ESO
        self.B = np.array([[0], [self.b], [0]])
        self.eso.B = np.array([[0], [self.b], [0]])
        pass

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot, i, M):
        ### TODO implement ADRC

        m = M[i,i]
        self.set_b(1/m)
        self.eso.update([x[0]], [self.prev_u])
        state = self.eso.get_state()
        q_hat = state[0,0]
        q_dot_hat = state[1,0]
        f = state[2,0]


        u_pd = self.loc_control.calculate_control(x[0], q_dot_hat, q_d,q_d_dot, q_d_ddot)
        v = u_pd + q_d_ddot

        u = (v - f)/ self.b
        
        
        self.prev_u = u
        return u
    
    # calculates l1, l2, l3 based on p
    # p must be bigger than 0 otherwise the obserwer will be unstable
    def calculate_ls(self, p):
        return np.array([[3*p], [3 * p ** 2], [p**3]])