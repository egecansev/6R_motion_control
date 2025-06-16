from robot import Robot, generate_joint_trajectory, visualize_trajectory
from utils import check_workspace, random_cartesian_pose, generate_random_obstacles
import numpy as np
import matplotlib.pyplot as plt
import random

robot = Robot()

# Define reasonable limits for your robot's workspace
x_range = (-0.6, 0.6)
y_range = (-0.6, 0.6)
z_range = (0.1, 0.6)  # avoid below-ground z

rpy_range_deg = (-180, 180)
print("Please select start and goal poses.")
while True:
    start_pose = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)
    end_pose   = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)
    # start_pose = [-0.05804681833543934, 0.5195282788249148, 0.46450439180398284, 72.70057560604963, 79.92798550917547, -75.13151167832743]
    # end_pose = [-0.31736948862316594, -0.4936821454316658, 0.2442967510681431, -24.124468841999715, -32.31823487198611, -35.84207166444298]
    # start_pose = [0.11883671139255181, -0.28969741185045106, 0.18428628863392274, 31.004938519887958, 164.83590493755548, -130.73508261938125]
    # end_pose = [0.39266481560570776, 0.3419827060942672, 0.4503110052328134, -89.5844053303092, 53.29334258767639, 1.953575315096515]
    # start_pose = [0.010202292134324664, -0.16733076354263393, 0.3881742612603736, -13.573202335727217, -149.11652805933522, -168.1781440432909]
    # end_pose = [-0.0012469455833961085, 0.19950280188920955, 0.2265028296337988, -4.348312891684998, -49.34550199649263, 9.160159508943792]
    # start_pose = [-0.010202292134324664, -0.16733076354263393, 0.3881742612603736, -13.573202335727217, -149.11652805933522, -168.1781440432909]
    # end_pose = [0.001202292134324664, 0.19733076354263393, 0.2265028296337988, -4.348312891684998, -49.34550199649263, 9.160159508943792]
    # start_pose = [-0.40023288735170537, 0.3710056200569316, 0.509513139568647, -37.366908022995375, 111.24864565899628, -170.562625252173]
    # end_pose = [0.4310279617716294, -0.23696263433554782, 0.42179494232712045, 126.70289290352565, -104.34758843993417, 78.94755997475363]
    # start_pose = [0.09711834210875364, -0.13977407455432644, 0.14580460873171855, -46.7901845565174, -32.34707952908667, 170.91769956125387]
    # end_pose = [0.27254387084843834, 0.21723085904661754, 0.3384045175734477, 169.3451630346135, -138.90210507311218, -130.35463565662553]
    # start_pose = [0.2986057691264906, -0.21287623669179123, 0.49670717553834565, -9.952817494536788, 76.76044508848923, -152.30137506054103]
    # end_pose = [0.22342671479261145, -0.04460715791012193, 0.19215167456978802, 145.6943792147389, 131.12732604039508, -119.01345979500289]
    # start_pose = [0.027897204353937444, -0.04131164919222857, 0.33053197897344666, 9.64692194168012, 61.18872855865001, -67.03688143824859]
    # end_pose = [-0.3494178246870699, -0.23223613693563952, 0.1752316130742211, 158.21525068885455, 122.77606579326624, -73.34990918964056]
    if check_workspace(start_pose, end_pose, robot):
        break

x_range_obs = ((start_pose[0] + end_pose[0]) / 4, 3 * (start_pose[0] + end_pose[0]) / 4)
y_range_obs = ((start_pose[1] + end_pose[1]) / 4, 3 * (start_pose[1] + end_pose[1]) / 4)
z_range_obs = ((start_pose[2] + end_pose[2]) / 4, 3 * (start_pose[2] + end_pose[2]) / 4)
print("Start Pose:", start_pose)
print("End Pose:", end_pose)
obstacles = generate_random_obstacles(x_range_obs, y_range_obs, z_range_obs)
cart_trajectory, trajectory, joints_fk = generate_joint_trajectory(start_pose, end_pose, robot.dh_param, obstacles)
joint_trajectory = np.array(trajectory)

if joint_trajectory.ndim != 2 or joint_trajectory.shape[0] == 0:
    print("No valid solution found to plot.")
else:

    if not np.allclose(joints_fk[0][0][-1], start_pose[:3], atol=1e-3):
        print("Joint trajectory does not start at the expected start pose!")
        print("Expected:", start_pose[:3])
        print("Got     :", joints_fk[0][0][-1])
    else:
        print("Joint trajectory starts at the expected pose.")
        if not np.allclose(joints_fk[-1][0][-1], end_pose[:3], atol=1e-3):
            print("Joint trajectory does not reach the expected goal pose!")
            print("Expected:", end_pose[:3])
            print("Got     :", joints_fk[-1][0][-1])
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
