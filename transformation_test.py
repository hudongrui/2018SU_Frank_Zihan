import transformations as tfs
import numpy as np

rot_matrix = 		tfs.rotation_matrix(0, (0, 0, 1))
rot_maxtrix_code =  tfs.rotation_matrix(-np.pi / 2.0, [0, 0, 1], [0, 0, 0]) 
quat1 = tfs.quaternion_from_matrix(rot_matrix)
quat2 = tfs.quaternion_from_matrix(rot_maxtrix_code)
# quat2 = tfs.quaternion_about_axis(np.pi/2.0, (0, 0, 1))

# q1 = tfs.quaternion_about_axis(np.pi, (1, 0, 0))
# q2 = tfs.quaternion_about_axis(-np.pi/2.0, (0, 0, 1))
print("rot: ", rot_matrix)
print("q1", quat1)
print("q2", quat2)
# print(q1)
# print(q2)
# print(np.pi)