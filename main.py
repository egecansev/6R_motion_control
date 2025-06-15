from robot import (Robot, fk, analytical_ik,
                   check_collisions, generate_cartesian_trajectory,
                   generate_joint_trajectory, visualize_trajectory)
import numpy as np
import matplotlib.pyplot as plt

robot = Robot()

# Get joint limits
# print("Joint limits:")
# for i, (min_angle, max_angle) in enumerate(robot.get_joint_limits()):
#     print(f"Joint {i+1}: {min_angle:.1f} rad to {max_angle:.1f} rad")

desired_pose = [0.6, 0.5, 0.2, 0, 0, 0]
solution = analytical_ik(desired_pose, robot.get_dh_params())
if solution is not None:
    np.set_printoptions(precision=4, suppress=True, floatmode='fixed', linewidth=200, formatter={'float_kind':lambda x: f"{x:.4f}"})
    print("solution:", np.array(solution))
else:
    print("IK solution not found or target is unreachable.")

start_pose = [-0.6, -0.5, 0.2, 0, 0, 0]
end_pose = [0.6, -0.3, 0.6, 0, 0, 0]
cart_trajectory, trajectory, joints_fk = generate_joint_trajectory(start_pose, end_pose, robot.dh_param)
joint_trajectory = np.array(trajectory)
for i in range(joint_trajectory.shape[1]):
    plt.plot(joint_trajectory[:, i], label=f'Joint {i+1}')
plt.legend()
plt.title("Joint angles over trajectory")
plt.xlabel("Step")
plt.ylabel("Angle (rad)")
plt.grid()
plt.show()
visualize_trajectory(joint_trajectory, cart_trajectory, joints_fk, robot.dh_param)
# while True:
#         user_input = input("Joint angles (deg): ")
#         if user_input.strip().lower() == 'q':
#             print("Exiting.")
#             break
#
#         parts = user_input.strip().split()
#         if len(parts) != 6:
#             print("Please enter exactly 6 numbers.")
#             continue
#
#         try:
#             deg_angles = list(map(float, parts))
#         except ValueError:
#             print("Invalid input, please enter numbers only.")
#             continue
#
#         rad_angles = np.radians(deg_angles)
#         joint_positions, rotation_axes = fk(robot.dh_param, rad_angles)
#
#         obstacles = [
#             {'position': desired_pose[:3], 'radius': 0.02}
#         ]
#         plot_robot(joint_positions, rotation_axes, obstacles=obstacles, title=f"Pose for {deg_angles} degrees")
