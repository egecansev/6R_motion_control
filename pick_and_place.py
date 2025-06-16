import numpy as np
from robot import Robot, generate_joint_trajectory, visualize_trajectory
from utils import check_workspace, random_cartesian_pose, generate_random_obstacles, get_home_pose


def run_pick_and_place():
    robot = Robot()

    # Define poses (pick, place, home)
    home_pose = np.zeros(6)
    home_pose[:3] = get_home_pose(robot)
    home_pose = [0.0, 0.3, 0.4, 0, 180, 0]
    pick_pose = [0.0, 0.7, 0.1, 0, 180, 0]
    place_pose = [0.3, 0.4, 0.1, 0, 180, 0]

    poses = [home_pose, pick_pose, place_pose, home_pose]
    obstacles = generate_random_obstacles(x_range=(0.1, 0.4), y_range=(-0.4, 0.4), z_range=(0.05, 0.3))

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
    visualize_trajectory(full_trajectory, all_cartesian, all_fk, obstacles=obstacles)


if __name__ == "__main__":
    run_pick_and_place()
