from robot import Robot, fk, plot_robot
import numpy as np


robot = Robot()

# Get joint limits
print("Joint limits:")
for i, (min_angle, max_angle) in enumerate(robot.get_joint_limits()):
    print(f"Joint {i+1}: {min_angle:.1f} rad to {max_angle:.1f} rad")


while True:
        user_input = input("Joint angles (deg): ")
        if user_input.strip().lower() == 'q':
            print("Exiting.")
            break

        parts = user_input.strip().split()
        if len(parts) != 6:
            print("Please enter exactly 6 numbers.")
            continue

        try:
            deg_angles = list(map(float, parts))
        except ValueError:
            print("Invalid input, please enter numbers only.")
            continue

        rad_angles = np.radians(deg_angles)
        joint_positions, rotation_axes = fk(robot.dh_param, rad_angles)
        plot_robot(joint_positions, rotation_axes, title=f"Pose for {deg_angles} degrees")
