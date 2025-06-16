import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from robot import analytical_ik, check_collisions, fk
from scipy.signal import savgol_filter

def get_jacobian(q, z):
    jacobian = np.zeros((6, 6))
    p_last = np.array(q[-1])

    for i in range(6):
        z_i = np.array(z[i])
        p_i = np.array(q[i])
        jacobian[0:3, i] = np.cross(z_i, p_last - p_i)
        jacobian[3:6, i] = z_i

    return jacobian

def inject_waypoint_around_dead_zone(start, end, dead_zone_radius= 0.2):
    # For example, offset a waypoint around the dead zone edge
    # Find midpoint between start and end
    mid = (start + end) / 2

    # Shift midpoint in XY plane away from base center along a vector
    offset_dir = np.array([mid[0], mid[1]])

    if np.linalg.norm(offset_dir) < 1e-6:
        # Midpoint is too close to base center, use a default safe direction
        offset_dir = np.array([1.0, 1.0])

    offset_dir = offset_dir / np.linalg.norm(offset_dir)
    offset_dist = dead_zone_radius * 2
    mid[:2] += offset_dir * offset_dist
    mid[2] += 0.05

    return mid

def generate_cartesian_trajectory(start_pose, end_pose, steps = 50):
    steps += 1
    int_pos = []
    poses = []
    pos_start = np.array(start_pose[:3])
    pos_end = np.array(end_pose[:3])
    dead_zone_radius = 0.2
    mid_point = np.zeros(3)

    # Check if the shortest route between start and end point passes through the dead-zone around base
    projection = (np.dot(- pos_start[:2], pos_end[:2] - pos_start[:2]) /
                np.dot(pos_end[:2] - pos_start[:2], pos_end[:2] - pos_start[:2]))
    closest = pos_start[:2] + projection * (pos_end[:2] - pos_start[:2])
    if np.linalg.norm(closest) < dead_zone_radius:
        mid_point = inject_waypoint_around_dead_zone(pos_start, pos_end)

    times = np.linspace(0, 1, steps)
    for t in times:
        if np.all(mid_point == 0):
            int_pos.append((1 - t) * pos_start + t * pos_end)
        else:
            # Quadratic Bezier curve
            int_pos.append((1 - t)**2 * pos_start + 2*(1 - t)*t * mid_point + t**2 * pos_end)



    rpy_start = np.array(start_pose[3:])
    rpy_end = np.array(end_pose[3:])

    # Use Spherical Linear Interpolation of Rotations (SLERP) for rotations
    key_rots = R.from_euler('xyz', [rpy_start, rpy_end], degrees= True)
    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)

    int_rot = slerp(times)

    for i in range(steps):
        pos = int_pos[i]
        rot_mat = int_rot[i].as_matrix()
        poses.append([*pos, rot_mat])
    return poses


def generate_joint_trajectory(start, end, dh, obstacles):
    cartesian_poses = generate_cartesian_trajectory(start, end, 50)
    # Separate positions and rotations
    positions = np.array([pose[:3] for pose in cartesian_poses])
    rotations = [pose[3] for pose in cartesian_poses]  # R is 3x3 matrix

    # Smooth only the positions
    smoothed_positions = savgol_filter(positions, window_length=7, polyorder=3, axis=0)

    # Recombine with original (or separately interpolated) rotation matrices
    smoothed_cartesian_poses = [[*smoothed_positions[i], rotations[i]] for i in range(len(cartesian_poses))]
    cartesian_poses = smoothed_cartesian_poses
    joint_trajectory = []
    failed_steps = []
    all_joint_positions = []
    weights = np.array([0.5, 2.0, 2.0, 2.0, 0.5, 0.5])
    #weights = np.ones(6)

    for pose in cartesian_poses:
        best_solution = None
        norm = 0
        candidates = analytical_ik(pose, dh)
        if joint_trajectory:
            for candidate in candidates:
                joint_positions, rotation_axes = fk(dh, candidate)
                if check_collisions(joint_positions, obstacles) == -1: # Check if there are collisions
                    new_distance = np.abs(np.array(joint_trajectory[-1]) - np.array(candidate))
                    new_norm = np.linalg.norm(weights * new_distance)
                    if best_solution is None or new_norm < norm:
                        norm = new_norm
                        best_solution = candidate
                        best_joint_positions = joint_positions
                        best_rotation_axes = rotation_axes
        else:
            # Check manipulability as initial selection criterion
            max_manipulability = 0
            for candidate in candidates:
                joint_positions, rotation_axes = fk(dh, candidate)
                if check_collisions(joint_positions, obstacles) == -1:
                    J = get_jacobian(joint_positions, rotation_axes)
                    manipulability = np.sqrt(np.linalg.det(J @ J.T))
                    if manipulability > max_manipulability:
                        max_manipulability = manipulability
                        best_solution = candidate
                        best_joint_positions = joint_positions
                        best_rotation_axes = rotation_axes
        if best_solution is None:
            print(f"Warning: No valid IK solution found for pose {pose}")
            failed_steps.append(pose)
            continue
        joint_trajectory.append(best_solution)
        all_joint_positions.append((best_joint_positions, best_rotation_axes))

    trajectory_np = np.array(joint_trajectory)  # shape (N, 6)

    # Apply Savitzky-Golay filter per joint
    smoothed = np.zeros_like(trajectory_np)
    for i in range(trajectory_np.shape[1]):
        smoothed[:, i] = savgol_filter(trajectory_np[:, i], window_length=7, polyorder=3)

    # Force start and end points to remain exact
    smoothed[0] = trajectory_np[0]
    smoothed[-1] = trajectory_np[-1]
    joint_trajectory = smoothed
    return cartesian_poses, joint_trajectory, all_joint_positions

