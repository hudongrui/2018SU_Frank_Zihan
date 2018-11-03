import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time


# ----------------------------------- basic -----------------------------------------
# Random waypoints used to obtain a random geometric path. Here,
# we use spline interpolation.
dof = 7
way_pts = np.random.randn(5, dof)
path = ta.SplineInterpolator(np.linspace(0, 1, 5), way_pts)

# Create velocity bounds, then velocity constraint object
vlim_ = np.random.rand(dof) * 20
vlim = np.vstack((-vlim_, vlim_)).T
# Create acceleration bounds, then acceleration constraint object
alim_ = np.random.rand(dof) * 2
alim = np.vstack((-alim_, alim_)).T
pc_vel = constraint.JointVelocityConstraint(vlim)
pc_acc = constraint.JointAccelerationConstraint(
    alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

# Setup a parametrization instance
instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')

# Retime the trajectory, only this step is necessary.
t0 = time.time()
jnt_traj, aux_traj = instance.compute_trajectory(0, 0)
print("Parameterization time: {:} secs".format(time.time() - t0))
ts_sample = np.linspace(0, jnt_traj.get_duration(), 100)
qdds_sample = jnt_traj.evaldd(ts_sample)

plt.plot(ts_sample, qdds_sample)
plt.xlabel("Time (s)")
plt.ylabel("Joint acceleration (rad/s^2)")
plt.legend(["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"])
plt.show()

# ----------------------------------- end of basic -----------------------------------

# ----------------------------------------- 1 -------------------------------------
# s_array = [0, 1, 2]
# wp_array = [(0, 0), (1, 2), (2, 0)]
# path = ta.SplineInterpolator(s_array, wp_array)
# s_sampled = np.linspace(0, 2, 100)
# q_sampled = path.eval(s_sampled)
# plt.plot(s_sampled, q_sampled)
# plt.show()
