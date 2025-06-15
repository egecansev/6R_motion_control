def check_collisions(joint_positions):
    # Check collisions with ground
    for i, joint in enumerate(joint_positions):
        if joint[2] < 0:
            print("Collision with ground on Joint", i + 1, "!")
            return i
    return -1