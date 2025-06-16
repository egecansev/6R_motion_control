import numpy as np

def get_closest_point_to_obstacle(start, end, obstacle):
    direction = end - start
    length_squared = np.dot(direction, direction)
    if length_squared == 0:
        return start  # start and end are the same point

    t = np.dot(obstacle - start, direction) / length_squared
    t = np.clip(t, 0, 1)  # Clamp to segment [0, 1]
    closest = start + t * direction
    return closest

def check_collisions(joint_positions, obstacles):
    # Check collisions with ground
    for i, joint in enumerate(joint_positions):
        if joint[2] < 0:
            return i

    link_radius = 0.02
    for obstacle in obstacles:
        obs = np.array(obstacle['position'])
        for i in range(len(joint_positions) - 1):
            a = np.array(joint_positions[i])
            b = np.array(joint_positions[i + 1])
            closest = get_closest_point_to_obstacle(a, b, obs)
            distance = np.linalg.norm(closest - obs)
            if distance < obstacle['radius'] + link_radius:
                print(f"Collision with obstacle at Link {i + 1}")
                return i
    return -1