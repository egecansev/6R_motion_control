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


def euler_to_matrix(pd):
    roll = np.deg2rad(pd[0])
    pitch = np.deg2rad(pd[1])
    yaw = np.deg2rad(pd[2])
    rot_mat = R.from_euler('xyz', [roll, pitch, yaw])
    return rot_mat.as_matrix()

def analytical_ik(pd, dh):
    solutions = []
    (px, py, pz) = pd[:3]

    r = pd[3]
    r11 = r[0,0]
    r12 = r[0, 1]
    r13 = r[0, 2]
    r21 = r[1, 0]
    r22 = r[1, 1]
    r23 = r[1, 2]
    r31 = r[2, 0]
    r32 = r[2, 1]
    r33 = r[2, 2]

    d1 = dh[0][2]
    a2 = dh[1][1]
    a3 = dh[2][1]
    d4 = dh[3][2]
    d5 = dh[4][2]
    d6 = dh[5][2]

    # IK solution from doi.org/10.3390/robotics11060137

    A = py - d6 * r23
    B = px - d6 * r13

    for sign1 in [-1, 1]:
        try:
            q1 = sign1 * np.arctan2(np.sqrt(B**2 + A**2 - d4**2), d4) + np.arctan2(B, -A)
            c1 = np.cos(q1)
            s1 = np.sin(q1)

            C = c1 * r11 + s1 * r21
            D = c1 * r22 - s1 * r12
            E = s1 * r11 - c1 * r21

            for sign5 in [-1, 1]:
                q5 = sign5 * np.arctan2(np.sqrt(E**2 + D**2), s1 * r13 - c1 * r23)
                c5 = np.cos(q5)
                s5 = np.sin(q5)

                if s5:
                    q6 = np.arctan2(D / s5, E / s5)
                else:
                    q6 = 0
                c6 = np.cos(q6)
                s6 = np.sin(q6)

                F = c5 * c6

                q234 = np.arctan2(r31 * F - s6 * C, F * C + s6 * r31)
                c234 = np.cos(q234)
                s234 = np.sin(q234)

                KC = c1 * px + s1 * py - s234 * d5 + c234 * d6
                KS = pz - d1 + c234 * d5 + s234 * s5 * d6

                c3 = (KS**2 + KC**2 - a2**2 - a3**2) / (2 * a2 * a3)
                if c3 > 1:
                    # print("No viable solutions for sign1:", sign1, "sign5:", sign5)
                    continue


                # plus-minus is wrong on the reference, should be on (27) instead of (28)
                for sign3 in [-1, 1]:
                    s3 = sign3 * np.sqrt(1 - c3 ** 2)
                    q3 = np.arctan2(s3, c3)

                    q2 = np.arctan2(KS, KC) - np.arctan2(s3 * a3, c3 * a3 + a2)

                    q4 = q234 - q2 - q3

                    # Offset for theta2 = pi / 2 in robot description
                    q2 = q2 + np.pi/2

                    solution = np.array([q1, q2, q3, q4, q5, q6])
                    # print("Solution for sign1:", sign1, "sign5:", sign5, "sign3:", sign3, ":", solution)
                    solutions.append(solution)
        except:
            continue
    return solutions



