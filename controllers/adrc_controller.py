import numpy as np
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class ADRController(Controller):
    def __init__(self, Tp, params):
        self.joint_controllers = []
        self.mani_model = ManiuplatorModel(Tp, 0.01, 0.04)
        for param in params:
            self.joint_controllers.append(ADRCJointController(*param, Tp))

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        u = []
        for i, controller in enumerate(self.joint_controllers):
            u.append(controller.calculate_control([x[i], x[i+2]], q_d[i], q_d_dot[i], q_d_ddot[i], i, self.mani_model.M(x)))
        u = np.array(u)[:, np.newaxis]
        return u

