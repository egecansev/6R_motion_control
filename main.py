from robot import (Robot, fk, analytical_ik,
                   check_collisions, generate_cartesian_trajectory,
                   generate_joint_trajectory, visualize_trajectory)
import numpy as np
import matplotlib.pyplot as plt

robot = Robot()

start_pose = [0.3, 0.5, 0.6, 90, 0, 0]
end_pose = [0.3, 0.5, 0.6, 0, 90, 0]
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
