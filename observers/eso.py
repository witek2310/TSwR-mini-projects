from copy import copy
import numpy as np


class ESO:
    def __init__(self, A, B, W, L, state, Tp):
        self.A = A
        self.B = B
        self.W = W
        self.L = L
        self.state = np.pad(np.array(state), (0, A.shape[0] - len(state)))
        self.Tp = Tp
        self.states = []


    def set_B(self, B):
        self.B = B

    def update(self, q, u):
        self.states.append(copy(self.state.reshape((1,self.A.shape[0])).tolist()[0]))
        ### TODO implement ESO update
        u = np.array([u])
        q = np.array([q]).reshape((len(q),1))
        self.state = np.array(self.state).reshape((self.A.shape[0],1))
        self.state = self.state + (self.A @ self.state + self.B @ u + self.L @ (q - self.W @ self.state)) * self.Tp


    def get_state(self):
        return self.state
