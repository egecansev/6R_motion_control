import numpy as np
import random
from robot import Robot, generate_joint_trajectory, visualize_trajectory
from utils import check_workspace, random_cartesian_pose, generate_random_obstacles, get_home_pose


def prompt_for_pose(name):
    print(f"Enter {name} pose as 6 values: x y z roll pitch yaw (in meters and degrees) \n"
          f"Example pick pose: 0.0 0.7 0.1 0 180 0 \n"
          f"Example place pose: 0.3 0.4 0.1 0 180 0")
    while True:
        try:
            values = list(map(float, input(f"{name} pose: ").strip().split()))
            if len(values) == 6:
                return values
            else:
                print("Please enter exactly 6 values.")
        except ValueError:
            print("Invalid input. Please enter 6 numeric values.")


def run_pick_and_place():
    robot = Robot()

    # Define reasonable limits for your robot's workspace
    x_range = (-0.5, 0.5)
    y_range = (-0.5, 0.5)
    z_range = (0.1, 0.7)  # avoid below-ground z

    rpy_range_deg = (-180, 180)

    # Define poses (pick, place, home)
    home_pose = np.zeros(6)
    home_pose[:3] = get_home_pose(robot)
    home_pose = [0.0, 0.3, 0.4, 0, 180, 0]

    # Ask user for obstacle inclusion
    use_obstacles = input("Do you want to include obstacles in the scene? (y/n): ").strip().lower() == 'y'

    # Ask user for pose input
    use_custom_poses = input("Do you want to input your own pick and place poses? (y/n): ").strip().lower() == 'y'

    # Get pick/place poses
    if use_custom_poses:
        pick_pose = prompt_for_pose("Pick")
        place_pose = prompt_for_pose("Place")
    else:
        pick_pose = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)
        place_pose = random_cartesian_pose(x_range, y_range, z_range, rpy_range_deg)

    while True:

        poses = [home_pose, pick_pose, place_pose, home_pose]

        obstacles = []
        if use_obstacles:
            if random.choice([True, False]):
                obstacles += generate_random_obstacles(home_pose, pick_pose)
            if random.choice([True, False]):
                obstacles += generate_random_obstacles(pick_pose, place_pose)
            if random.choice([True, False]):
                obstacles += generate_random_obstacles(place_pose, home_pose)

        all_trajectories = []
        all_cartesian = []
        all_fk = []

        for i in range(len(poses) - 1):
            start = poses[i]
            end = poses[i+1]
            if not check_workspace(start, end, robot):
                print(f"Segment {i}: Workspace violation between start and end poses.")
                return

            cartesian, trajectory, fk_data = generate_joint_trajectory(start, end, robot.dh_param, obstacles)
            if len(trajectory) == 0:
                print(f"Segment {i}: Trajectory generation failed.")
                return

            all_trajectories.append(trajectory)
            all_cartesian.extend(cartesian)
            all_fk.extend(fk_data)

        # Combine all motion segments
        full_trajectory = np.vstack(all_trajectories)

        print("Executing pick and place trajectory...")
        visualize_trajectory(full_trajectory, all_cartesian, all_fk, obstacles=obstacles,
                             animate_object=True,
                             pick_index=len(all_cartesian) // 3,
                             place_index=2 * len(all_cartesian) // 3,
                             object_init_pos=pick_pose[:3])


if __name__ == "__main__":
    run_pick_and_place()
