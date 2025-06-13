import numpy as np
from scipy.spatial.transform import Rotation as R

def fk(dh, q):
    pose = np.eye(4)
    tf_matrices = []
    joint_positions = [(0.0, 0.0, 0.0)]
    rotation_axes = []
    transformation_matrix = []
    for i, (theta, a, d, alpha) in enumerate(dh):
        ct = np.cos(theta + q[i])
        st = np.sin(theta + q[i])
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        transformation_matrix = np.array([
            [ct, -ca * st, sa * st, a * ct],
            [st, ca * ct, -sa * ct, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        tf_matrices.append(transformation_matrix)
        rotation_vector = pose[:3, 2]
        rotation_direction = rotation_vector / np.linalg.norm(rotation_vector)
        rotation_axes.append(rotation_direction)
        pose = np.dot(pose, transformation_matrix)
        joint_positions.append((pose[0,3], pose[1,3], pose[2,3]))

        #Last element of joint_positions belong to end-effector

    return joint_positions, rotation_axes