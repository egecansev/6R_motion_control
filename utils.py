import numpy as np
from random import randint
from robot import fk


def rotation_matrix_to_rpy(R):
    """
    Convert a rotation matrix to roll-pitch-yaw (XYZ convention).
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll  = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0

    return np.degrees([roll, pitch, yaw])

def check_workspace(start_pose, end_pose, robot):
    outer = robot.outer_limit
    inner = robot.inner_limit
    start_range = np.linalg.norm(start_pose[:3])
    end_range = np.linalg.norm(end_pose[:3])
    if start_range > outer:
        if end_range > outer:
            print("Both selected poses are out of workspace! Please select new poses.")
        else:
            print("First selected pose is out of workspace! Please select a new pose.")
    else:
        if end_range > outer:
            print("Second selected pose is out of workspace! Please select a new pose.")
        else:
            start_range = np.linalg.norm(start_pose[:2])
            end_range = np.linalg.norm(end_pose[:2])
            if start_range < inner:
                if end_range < inner:
                    print("Beware! Both selected poses are beyond inner workspace limit! Unexpected behavior might occur!")
                else:
                    print("Beware! First selected pose is beyond inner workspace limit! Unexpected behavior might occur!")
            else:
                if end_range < inner:
                    print("Beware! Second selected pose is beyond inner workspace limit! Unexpected behavior might occur!")
            return True
    return False

def random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg):
    x = np.random.uniform(*x_range)
    y = np.random.uniform(*y_range)
    z = np.random.uniform(*z_range)

    roll = np.random.uniform(*rpy_range_deg)
    pitch = np.random.uniform(*rpy_range_deg)
    yaw = np.random.uniform(*rpy_range_deg)

    return [x, y, z, roll, pitch, yaw]

def generate_random_obstacles(x_range, y_range, z_range, radius_range=(0.05, 0.2)):
    """
    Generate a list of random obstacles within given bounds.

    Args:
        num_obstacles (int): Number of obstacles to generate.
        workspace_bounds (dict): Dictionary with 'x', 'y', 'z' bounds as (min, max) tuples.
        radius_range (tuple): Minimum and maximum radius of obstacles.

    Returns:
        List[Dict]: Each dict has 'position' and 'radius'.
    """
    obstacles = []
    num_obstacles = randint(0,1)
    for _ in range(num_obstacles):
        x = np.random.uniform(*x_range)
        y = np.random.uniform(*y_range)
        z = np.random.uniform(*z_range)
        radius = np.random.uniform(*radius_range)

        obstacle = {
            'position': [x, y, z],
            'radius': radius
        }
        obstacles.append(obstacle)

    return obstacles

def get_home_pose(robot):
    jp, ra = fk(robot.dh_param, np.zeros(6))
    xyz = jp[-1]  # End-effector position
    return list(xyz)