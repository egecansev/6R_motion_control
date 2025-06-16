import numpy as np

class Joint:
    def __init__(self, name, min_limit, max_limit):
        self.name = name
        self.min_limit = min_limit
        self.max_limit = max_limit


class Robot:
    def __init__(self):
        self.Joints = [
            Joint("joint1", -np.pi, np.pi),
            Joint("joint2", -np.pi, np.pi),
            Joint("joint3", -np.pi, np.pi),
            Joint("joint4", -np.pi, np.pi),
            Joint("joint5", -np.pi, np.pi),
            Joint("joint6", -np.pi, np.pi)
        ]

        # DH parameters: [theta, a, d, alpha] from UR5
        self.dh_param = [
            [0, 0, 0.089159, np.pi / 2],
            [-np.pi / 2, -0.425, 0, 0],
            [0, -0.39225, 0, 0],
            [0, 0, 0.10915, np.pi / 2],
            [0, 0, 0.09465, -np.pi / 2],
            [0, 0, 0.0823, 0]
        ]

        self.outer_limit = 0.8
        self.inner_limit = 0.2

    def get_joint_limits(self):
        joint_limits = []
        for joint in self.Joints:
            joint_limits.append((joint.min_limit, joint.max_limit))
        return joint_limits

    def get_dh_params(self):
        return self.dh_param