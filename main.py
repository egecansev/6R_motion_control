import numpy as np
import matplotlib.pyplot as plt


from robot import Robot, generate_joint_trajectory, visualize_trajectory
from utils import check_workspace, random_cartesian_pose, generate_random_obstacles


def prompt_for_pose(name):
    print(f"Enter {name} pose as 6 values: x y z roll pitch yaw (in meters and degrees) \n"
          f"Example start pose: 0.0 0.7 0.1 0 180 0 \n"
          f"Example goal pose: 0.3 0.4 0.1 0 180 0")
    while True:
        try:
            values = list(map(float, input(f"{name} pose: ").strip().split()))
            if len(values) == 6:
                return values
            else:
                print("Please enter exactly 6 values.")
        except ValueError:
            print("Invalid input. Please enter 6 numeric values.")


def main():
    robot = Robot()

    # Define workspace limits
    x_range = (-0.6, 0.6)
    y_range = (-0.6, 0.6)
    z_range = (0.1, 0.6)
    rpy_range_deg = (-180, 180)

    # Ask user for pose input
    use_custom_poses = input("Do you want to input your own start and goal poses? (y/n): ").strip().lower() == 'y'

    # Ask user whether to include obstacles
    use_obstacles = input("Do you want to include obstacles in the scene? (y/n): ").strip().lower() == 'y'

    # Get start and end poses
    if use_custom_poses:
        start_pose = prompt_for_pose("Start")
        end_pose = prompt_for_pose("Goal")
    else:
        while True:
            start_pose = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)
            end_pose = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)
            if check_workspace(start_pose, end_pose, robot):
                break

    print("Start Pose:", start_pose)
    print("End Pose:", end_pose)

    # Generate obstacles if requested
    obstacles = generate_random_obstacles(start_pose, end_pose) if use_obstacles else []

    # Generate trajectory
    cart_trajectory, trajectory, joints_fk = generate_joint_trajectory(start_pose, end_pose, robot.dh_param, obstacles)
    joint_trajectory = np.array(trajectory)

    if joint_trajectory.ndim != 2 or joint_trajectory.shape[0] == 0:
        print("No valid solution found to plot.")
        return

    # Check trajectory accuracy
    start_fk = joints_fk[0][0][-1]
    end_fk = joints_fk[-1][0][-1]



    # Plot joint angle trajectories
    for i in range(joint_trajectory.shape[1]):
        plt.plot(joint_trajectory[:, i], label=f'Joint {i+1}')
    #     print(f'Joint {i+1}:', joint_trajectory[:, i])

    plt.legend()
    plt.title("Joint angles over trajectory")
    plt.xlabel("Step")
    plt.ylabel("Angle (rad)")
    plt.grid()
    plt.show()

    if not np.allclose(start_fk, start_pose[:3], atol=1e-3):
        print("Joint trajectory does not start at the expected start pose!")
        print("Expected:", start_pose[:3])
        print("Got     :", start_fk)
    else:
        print("Joint trajectory starts at the expected pose.")

    if not np.allclose(end_fk, end_pose[:3], atol=1e-3):
        print("Joint trajectory does not reach the expected goal pose!")
        print("Expected:", end_pose[:3])
        print("Got     :", end_fk)
    else:
        if len(joint_trajectory) < 50:
            print("Joint trajectory reaches at the expected pose with missing steps!")
        else:
            print("Joint trajectory reaches at the expected pose.")

    # Visualize the full motion
    visualize_trajectory(joint_trajectory, cart_trajectory, joints_fk, obstacles=obstacles)


if __name__ == "__main__":
    main()