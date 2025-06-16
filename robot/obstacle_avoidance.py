import numpy as np


def check_collisions(joint_positions, obstacles):
    # Check collisions with ground
    for i, joint in enumerate(joint_positions):
        if joint[2] < 0:
            #print("Collision with ground on Joint", i + 1, "!")
            return i

    # Check collisions with obstacles
    link_radius = 0.02
    for obstacle in obstacles:
        p = np.array(obstacle['position'])
        for i in range(len(joint_positions) - 1):
            a = np.array(joint_positions[i])
            b = np.array(joint_positions[i + 1])
            distance = np.dot(p - a, b - a) / np.dot(b - a, b - a)
            if distance < obstacle['radius'] + link_radius:
                print(f"Collision with obstacle at Link {i + 1}")
                return i
    return -1