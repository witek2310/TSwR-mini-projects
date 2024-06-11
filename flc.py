import matplotlib.pyplot as plt
import numpy as np

from controllers.dummy_controller import DummyController
from controllers.feedback_linearization_controller import FeedbackLinearizationController
from trajectory_generators.constant_torque import ConstantTorque
from trajectory_generators.sinusonidal import Sinusoidal
from trajectory_generators.poly3 import Poly3
from utils.simulation import simulate

Tp = 0.01
start = 0
end = 6

"""
Switch to FeedbackLinearizationController as soon as you implement it
"""
controller = FeedbackLinearizationController(Tp, 0.1, 1)
# controller = DummyController(Tp)

"""
Here you have some trajectory generators. You can use them to check your implementations.
At the end implement Point2point trajectory generator to move your manipulator to some desired state.
"""
# traj_gen = ConstantTorque(np.array([0., 1.0])[:, np.newaxis])
# traj_gen = Sinusoidal(np.array([0., 1.]), np.array([2., 2.]), np.array([0., 0.]))
traj_gen = Poly3(np.array([0., 0.]), np.array([np.pi, np.pi/6]), end)


Q, Q_d, u, T = simulate("PYBULLET", traj_gen, controller, Tp, end)


"""
You can add here some plots of the state 'Q' (consists of q and q_dot), controls 'u', desired trajectory 'Q_d'
with respect to time 'T' to analyze what is going on in the system
"""
print(Q.shape)
plt.subplot(221)
plt.plot(T, Q[:, 0], 'r', label='traj')
plt.plot(T, Q_d[:, 0], 'b', label="traj_d")
plt.plot(T, Q[:, 2], label='traj_dot')
plt.plot(T, Q_d[:, 2], label="traj_d_dot")
plt.legend()
plt.subplot(222)
plt.plot(T, Q[:, 1], 'r', label = 'traj')
plt.plot(T, Q_d[:, 1], 'b', label = "traj_d")
plt.plot(T, Q[:, 3], label = 'traj_dot')
plt.plot(T, Q_d[:, 3], label = "traj_d_dot")
plt.legend()
plt.subplot(223)
plt.plot(T, u[:, 0], 'r', label = "u_0")
plt.plot(T, u[:, 1], 'b', label = "u_1")
plt.legend()
plt.show()
