from robot import Robot, fk, plot_robot, analytical_ik
import numpy as np

from robot.kinematics import analytical_ik

robot = Robot()

# Get joint limits
print("Joint limits:")
for i, (min_angle, max_angle) in enumerate(robot.get_joint_limits()):
    print(f"Joint {i+1}: {min_angle:.1f} rad to {max_angle:.1f} rad")

desired_pose = [0.8, 0.3, 0.2, 0, 0, 0]
solution = analytical_ik(desired_pose, robot.get_dh_params())
if solution is not None:
    np.set_printoptions(precision=4, suppress=True, floatmode='fixed', linewidth=200, formatter={'float_kind':lambda x: f"{x:.4f}"})
    print(np.array(solution))
else:
    print("IK solution not found or target is unreachable.")

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

        obstacles = [
            {'position': desired_pose[:3], 'radius': 0.02}
        ]
        plot_robot(joint_positions, rotation_axes, obstacles=obstacles, title=f"Pose for {deg_angles} degrees")
