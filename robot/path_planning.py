import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from robot import analytical_ik, check_collisions, fk



def generate_cartesian_trajectory(start_pose, end_pose, steps = 50):
    int_pos = []
    poses = []
    pos_start = np.array(start_pose[:3])
    pos_end = np.array(end_pose[:3])
    rpy_start = np.array(start_pose[3:])
    rpy_end = np.array(end_pose[3:])

    # Use Spherical Linear Interpolation of Rotations (SLERP) for rotations
    key_rots = R.from_euler('xyz', [rpy_start, rpy_end])
    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)

    times = np.linspace(0, 1, steps)
    for t in times:
        int_pos.append((1 - t) * pos_start + t * pos_end)
    int_rot = slerp(times)
    int_rpy = int_rot.as_euler('xyz', degrees=True)

    for i in range(steps):
        pos = int_pos[i]
        rpy = int_rpy[i]
        poses.append([*pos, *rpy])
    return poses


def generate_joint_trajectory(start, end, dh):
    cartesian_poses = generate_cartesian_trajectory(start, end, 50)
    joint_trajectory = []
    failed_steps = []
    all_joint_positions = []

    for pose in cartesian_poses:
        best_solution = None
        norm = 0
        candidates = analytical_ik(pose, dh)
        if joint_trajectory:
            for candidate in candidates:
                joint_positions, rotation_axes = fk(dh, candidate)
                if check_collisions(joint_positions) == -1: # Check if there are collisions

                    new_distance = np.abs(np.array(joint_trajectory[-1]) - np.array(candidate))
                    new_norm = np.linalg.norm(new_distance)
                    if best_solution is None or new_norm < norm:
                        norm = new_norm
                        best_solution = candidate
        else:
            # TODO: Initial selection criteria
            for candidate in candidates:
                joint_positions, rotation_axes = fk(dh, candidate)
                if check_collisions(joint_positions) == -1:
                    best_solution = candidate
                    break
        if best_solution is None:
            print(f"Warning: No valid IK solution found for pose {pose}")
            failed_steps.append(pose)
            continue
        joint_trajectory.append(best_solution)
        all_joint_positions.append((joint_positions, rotation_axes))
    return cartesian_poses, joint_trajectory, all_joint_positions

