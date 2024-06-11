import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp, Kd, Kp):
        self.model = ManiuplatorModel(Tp, 0.1, 0.05)
        self.Kd = Kd
        self.Kp = Kp

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        
        q = np.array([x[0:2]]).reshape((2,1))
        q_dot = np.array([x[2:]]).reshape((2,1))

        # v = np.array([q_r_ddot]).reshape((2,1))  #open loop control
        v = np.array([q_r_ddot]).reshape((2,1)) + self.Kd * (np.array([q_r_dot]).reshape((2,1)) - q_dot) + self.Kp * ( np.array([q_r]).reshape(2,1) - q)
        tau = self.model.M(x) @ v + self.model.C(x) @ q_dot
        return tau

