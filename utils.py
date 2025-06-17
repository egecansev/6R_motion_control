import numpy as np
from robot import fk


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
                    print("Beware! Both selected poses are inside inner workspace limit! Unexpected behavior might occur!")
                else:
                    print("Beware! First selected pose is inside inner workspace limit! Unexpected behavior might occur!")
            else:
                if end_range < inner:
                    print("Beware! Second selected pose is inside inner workspace limit! Unexpected behavior might occur!")
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


def generate_random_obstacles(start_pose, end_pose, radius_range=(0.05, 0.2)):

    x_range = ((start_pose[0] + end_pose[0]) / 4, 3 * (start_pose[0] + end_pose[0]) / 4)
    y_range = ((start_pose[1] + end_pose[1]) / 4, 3 * (start_pose[1] + end_pose[1]) / 4)
    z_range = ((start_pose[2] + end_pose[2]) / 4, 3 * (start_pose[2] + end_pose[2]) / 4)

    obstacles = []
    num_obstacles = 1
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