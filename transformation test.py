import transformations as tfs
import numpy as np

rot_matrix = tfs.rotation_matrix(np.pi/2.0, (0, 0, 1))
quat1 = tfs.quaternion_from_matrix(rot_matrix)
quat2 = tfs.quaternion_about_axis(np.pi/2.0, (0, 0, 1))

q1 = tfs.quaternion_about_axis(np.pi, (1, 0, 0))
q2 = tfs.quaternion_about_axis(-np.pi/2.0, (0, 0, 1))
print(rot_matrix)
print(quat1)
print(quat2)
