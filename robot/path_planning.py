import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from robot import analytical_ik, check_collisions, fk, get_closest_point_to_obstacle
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

def get_closest_point_to_base(start, end):

    # Only consider 2D XY plane for projection
    projection_of_base = (np.dot(- start[:2], end[:2] - start[:2]) /
                  np.dot(end[:2] - start[:2], end[:2] - start[:2]))

    # 3D closest point to base
    closest_to_base = start[:3] + projection_of_base * (end[:3] - start[:3])

    return closest_to_base


def inject_waypoint_around_deadzone(closest, dead_zone_radius= 0.2):

    waypoint = np.zeros(3)
    # Determine offset direction from the base
    offset_dir = np.array([closest[0], closest[1]])

    if np.linalg.norm(offset_dir) < 1e-6:
        # Midpoint is too close to base center, use a default safe direction
        offset_dir = np.array([1.0, 1.0])

    offset_dir = offset_dir / np.linalg.norm(offset_dir)
    offset_dist = dead_zone_radius * 2
    waypoint[:2] = closest[:2] + offset_dir * offset_dist
    waypoint[2] = closest[2] + 0.05

    return waypoint

def inject_waypoint_around_obstacle(obs_position, obs_radius, closest):

    # Determine offset direction from the base
    offset_dir = np.array(closest - obs_position)
    offset_norm = np.linalg.norm(offset_dir)

    if offset_norm < 1e-6:
        # Midpoint is too close to base center, use a default safe direction
        offset_dir = np.array([1.0, 1.0, 1.0])

    offset_dir = offset_dir / offset_norm
    if offset_norm < obs_radius:
        offset_dist = obs_radius * 2
    else:
        offset_dist = offset_norm * 2
    waypoint = closest + offset_dir * offset_dist

    return waypoint

def generate_cartesian_trajectory(start_pose, end_pose, steps = 50, obstacles = None):
    steps += 1
    int_pos = []
    poses = []
    pos_start = np.array(start_pose[:3])
    pos_end = np.array(end_pose[:3])
    dead_zone_radius = 0.2
    mid_point = np.zeros(3)

    # Check if the shortest route between start and end point passes through the dead-zone around base

    closest = get_closest_point_to_base(pos_start, pos_end)
    if np.linalg.norm(closest[:2]) < dead_zone_radius:
        mid_point = inject_waypoint_around_deadzone(closest)

    # Check if the shortest route between start and end point passes through or around an obstacle
    for obs in obstacles:
        obs_pos = np.array(obs['position'])
        closest = get_closest_point_to_obstacle(pos_start, pos_end, obs_pos)
        dist_to_path = np.linalg.norm(closest - obs_pos)

        if dist_to_path < obs['radius'] * 1.2:  # Define a safe buffer
            mid_point = inject_waypoint_around_obstacle(obs_pos, obs['radius'], closest)
            break  # Inject one waypoint for now — extend later for multiple

    # TODO: Multiple obstacles near dead-zone

    times = np.linspace(0, 1, steps)
    for t in times:
        if np.all(mid_point == 0):
            int_pos.append((1 - t) * pos_start + t * pos_end)
        else:
            # Quadratic Bezier curve
            int_pos.append((1 - t)**2 * pos_start + 2*(1 - t)*t * mid_point + t**2 * pos_end)

    # TODO: Update Bezier curve when multiple midpoints


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
    cartesian_poses = generate_cartesian_trajectory(start, end, 50, obstacles)
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

    # Weighted norm to minimize the motion by joints 2, 3, 4
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
    if len(trajectory_np) >= 7:
        smoothed = np.zeros_like(trajectory_np)
        for i in range(trajectory_np.shape[1]):
            smoothed[:, i] = savgol_filter(trajectory_np[:, i], window_length=7, polyorder=3)

        # Force start and end points to remain exact
        smoothed[0] = trajectory_np[0]
        smoothed[-1] = trajectory_np[-1]
        joint_trajectory = smoothed

    return cartesian_poses, joint_trajectory, all_joint_positions

