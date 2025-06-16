from robot import (Robot, fk, analytical_ik, plot_robot,
                   check_collisions, generate_cartesian_trajectory,
                   generate_joint_trajectory, visualize_trajectory)
import numpy as np
import matplotlib.pyplot as plt
import random


def check_workspace(start_pose, end_pose):
    outer = robot.outer_limit
    inner = robot.inner_limit
    start_range = np.linalg.norm(start_pose[:3])
    end_range = np.linalg.norm(end_pose[:3])
    if start_range > outer:
        if end_range > outer:
            print("Both selected poses are out of workspace! Please select new poses.")
        else:
            print("First selected pose is out of workspace! Please select a new pose.")
    else:
        if end_range > outer:
            print("Second selected pose is out of workspace! Please select a new pose.")
        else:
            start_range = np.linalg.norm(start_pose[:2])
            end_range = np.linalg.norm(end_pose[:2])
            if start_range < inner:
                if end_range < inner:
                    print("Beware! Both selected poses are beyond inner workspace limit! Unexpected behavior might occur!")
                else:
                    print("Beware! First selected pose is beyond inner workspace limit! Unexpected behavior might occur!")
            else:
                if end_range < inner:
                    print("Beware! Second selected pose is beyond inner workspace limit! Unexpected behavior might occur!")
            return True
    return False



def random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg):
    x = np.random.uniform(*x_range)
    y = np.random.uniform(*y_range)
    z = np.random.uniform(*z_range)

    roll = np.random.uniform(*rpy_range_deg)
    pitch = np.random.uniform(*rpy_range_deg)
    yaw = np.random.uniform(*rpy_range_deg)

    return [x, y, z, roll, pitch, yaw]


def generate_random_obstacles(x_range, y_range, z_range, radius_range=(0.02, 0.08)):
    """
    Generate a list of random obstacles within given bounds.

    Args:
        num_obstacles (int): Number of obstacles to generate.
        workspace_bounds (dict): Dictionary with 'x', 'y', 'z' bounds as (min, max) tuples.
        radius_range (tuple): Minimum and maximum radius of obstacles.

    Returns:
        List[Dict]: Each dict has 'position' and 'radius'.
    """
    obstacles = []
    num_obstacles = random.randint(0,0)
    for _ in range(num_obstacles):
        x = np.random.uniform(*x_range)
        y = np.random.uniform(*y_range)
        z = np.random.uniform(*z_range)
        radius = np.random.uniform(*radius_range)

        obstacle = {
            'position': [x, y, z],
            'radius': radius
        }
        obstacles.append(obstacle)

    return obstacles


robot = Robot()

# Define reasonable limits for your robot's workspace
x_range = (-0.4, 0.4)
y_range = (-0.4, 0.4)
z_range = (0.1, 0.6)  # avoid below-ground z

rpy_range_deg = (-180, 180)
print("Please select start and goal poses.")
while True:
    start_pose = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)
    end_pose   = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)
    # start_pose = [0.010202292134324664, -0.16733076354263393, 0.3881742612603736, -13.573202335727217,
    #               -149.11652805933522, -168.1781440432909]
    # end_pose = [-0.0012469455833961085, 0.19950280188920955, 0.2265028296337988, -4.348312891684998, -49.34550199649263,
    #             9.160159508943792]
    # start_pose = [-0.010202292134324664, -0.16733076354263393, 0.3881742612603736, -13.573202335727217,
    #               -149.11652805933522, -168.1781440432909]
    # end_pose = [0.010202292134324664, 0.16733076354263393, 0.2265028296337988, -4.348312891684998, -49.34550199649263,
    #             9.160159508943792]
    # start_pose = [-0.40023288735170537, 0.3710056200569316, 0.509513139568647, -37.366908022995375, 111.24864565899628, -170.562625252173]
    # end_pose = [0.4310279617716294, -0.23696263433554782, 0.42179494232712045, 126.70289290352565, -104.34758843993417, 78.94755997475363]
    # start_pose = [0.09711834210875364, -0.13977407455432644, 0.14580460873171855, -46.7901845565174, -32.34707952908667, 170.91769956125387]
    # end_pose = [0.27254387084843834, 0.21723085904661754, 0.3384045175734477, 169.3451630346135, -138.90210507311218, -130.35463565662553]
    # start_pose = [0.2986057691264906, -0.21287623669179123, 0.49670717553834565, -9.952817494536788, 76.76044508848923, -152.30137506054103]
    # end_pose = [0.22342671479261145, -0.04460715791012193, 0.19215167456978802, 145.6943792147389, 131.12732604039508, -119.01345979500289]
    # start_pose = [0.027897204353937444, -0.04131164919222857, 0.33053197897344666, 9.64692194168012, 61.18872855865001, -67.03688143824859]
    # end_pose = [-0.3494178246870699, -0.23223613693563952, 0.1752316130742211, 158.21525068885455, 122.77606579326624, -73.34990918964056]
    if check_workspace(start_pose, end_pose):
        break


print("Start Pose:", start_pose)
print("End Pose:", end_pose)
obstacles = generate_random_obstacles(x_range, y_range, z_range)
# jp, rax = fk(robot.dh_param, np.zeros(6))
# plot_robot(jp, rax, obstacles=obstacles, title=f"Pose for {rax} degrees")
cart_trajectory, trajectory, joints_fk = generate_joint_trajectory(start_pose, end_pose, robot.dh_param, obstacles)
joint_trajectory = np.array(trajectory)
for i in range(joint_trajectory.shape[1]):
    plt.plot(joint_trajectory[:, i], label=f'Joint {i+1}')
    print(f'Joint {i+1}:', joint_trajectory[:, i])
plt.legend()
plt.title("Joint angles over trajectory")
plt.xlabel("Step")
plt.ylabel("Angle (rad)")
plt.grid()
plt.show()




visualize_trajectory(joint_trajectory, cart_trajectory, joints_fk, obstacles= obstacles)
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
