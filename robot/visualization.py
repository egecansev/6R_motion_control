import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

def plot_robot(joint_positions, rotation_axes=None, ax=None, obstacles=None, title="Robot Pose"):


    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')


    # Plot robot links
    for i in range(len(joint_positions) - 1):
        p1 = joint_positions[i]
        p2 = joint_positions[i + 1]
        ax.plot(
            [p1[0], p2[0]],
            [p1[1], p2[1]],
            [p1[2], p2[2]],
            color='gray',
            linewidth=2,
            label='Link' if i == 0 else ""
        )

    # Draw joints as cylinders around the rotation axes
    for i in range(len(joint_positions) - 1):
        origin = np.array(joint_positions[i])
        axis = np.array(rotation_axes[i])
        cyl_length = 0.05

        # Start and end points of the "cylinder"
        cyl_start = origin - axis * (cyl_length / 2)
        cyl_end = origin + axis * (cyl_length / 2)

        # Plot the cylinder
        ax.plot(
            [cyl_start[0], cyl_end[0]],
            [cyl_start[1], cyl_end[1]],
            [cyl_start[2], cyl_end[2]],
            color='red',
            linewidth=6,
            solid_capstyle='round'
        )

    # Add labels near each joint
    for i in range(len(joint_positions) - 1):
        origin = np.array(joint_positions[i])
        axis = np.array(rotation_axes[i])
        label_pos = origin + 0.03 * axis
        ax.text(
            label_pos[0], label_pos[1], label_pos[2],
            f'$q_{{{i + 1}}}$',
            color='black',
            fontsize=10
        )

    # Mark end-effector and draw a tool shape (a line sticking out)
    tool_tip = np.array(joint_positions[-1])
    #print(tool_tip)

    # Create a mock direction same as last link
    tool_direction = tool_tip - np.array(joint_positions[-2]) # Direction of last link
    tool_direction = tool_direction / np.linalg.norm(tool_direction)


    # Draw an "X" to represent the gripper
    grip_size = 0.02

    # Add orthogonal lines in the plane perpendicular to tool_direction
    arbitrary = np.array([1, 0, 0]) if not np.allclose(tool_direction, [1, 0, 0]) else np.array([0, 1, 0])
    x_arm = np.cross(tool_direction, arbitrary)
    x_arm = x_arm / np.linalg.norm(x_arm) * grip_size

    y_arm = np.cross(tool_direction, x_arm)
    y_arm = y_arm / np.linalg.norm(y_arm) * grip_size

    # First line of the "X"
    ax.plot(
        [tool_tip[0] - x_arm[0], tool_tip[0] + x_arm[0]],
        [tool_tip[1] - x_arm[1], tool_tip[1] + x_arm[1]],
        [tool_tip[2] - x_arm[2], tool_tip[2] + x_arm[2]],
        color='green',
        linewidth=2
    )

    # Second line of the "X"
    ax.plot(
        [tool_tip[0] - y_arm[0], tool_tip[0] + y_arm[0]],
        [tool_tip[1] - y_arm[1], tool_tip[1] + y_arm[1]],
        [tool_tip[2] - y_arm[2], tool_tip[2] + y_arm[2]],
        color='green',
        linewidth=2
    )

    # Obstacles to be added for Task 5
    if obstacles:
        for obs in obstacles:
            ax.scatter(*obs['position'], color='purple', s=100 * obs['radius'], label='Obstacle')

    # Ensure equal axis scaling by adjusting the limits
    all_points = np.array(joint_positions + [tool_tip])
    if obstacles:
        obs_points = np.array([obs['position'] for obs in obstacles])
        all_points = np.vstack([all_points, obs_points])

    x_limits = (np.min(all_points[:, 0]), np.max(all_points[:, 0]))
    y_limits = (np.min(all_points[:, 1]), np.max(all_points[:, 1]))
    z_limits = (np.min(all_points[:, 2]), np.max(all_points[:, 2]))

    # Calculate max range
    max_range = max(
        x_limits[1] - x_limits[0],
        y_limits[1] - y_limits[0],
        z_limits[1] - z_limits[0]
    ) / 2

    # Midpoints
    mid_x = np.mean(x_limits)
    mid_y = np.mean(y_limits)
    mid_z = np.mean(z_limits)

    # Set limits so all axes have same length
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    ax.legend()
    ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
    #plt.show()

def visualize_trajectory(joint_trajectory, cartesian_trajectory, all_joint_positions,
                         obstacles=None, animate_object=False, pick_index=None,
                         place_index=None, object_init_pos=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ee_x = [p[0] for p in cartesian_trajectory]
    ee_y = [p[1] for p in cartesian_trajectory]
    ee_z = [p[2] for p in cartesian_trajectory]

    if animate_object and object_init_pos:
        object_pos = object_init_pos.copy()
    else:
        object_pos = None
    carrying_object = False
    object_color = ['cyan']


    def update(frame):
        ax.cla()

        # Clamp frame to length to avoid index errors if frame is out of range
        f = min(frame, len(all_joint_positions) - 1)

        joint_positions, rotation_axes = all_joint_positions[f]

        nonlocal object_pos, carrying_object

        # Determine carrying state and object color
        if pick_index is not None and place_index is not None:
            if f == 0:
                start_time = time.time()
                while True:
                    if time.time() - start_time > 1:
                        break
            elif f < pick_index:
                carrying_object = False
                object_color[0] = 'cyan'
                object_pos = object_init_pos
            elif f == pick_index:
                carrying_object = False  # just touching or picking
                start_time = time.time()
                while True:
                    if time.time() - start_time > 1:
                        break
            elif pick_index <= f < place_index:
                carrying_object = True
                object_color[0] = 'green'
            elif f == place_index:
                carrying_object = True
                object_color[0] = 'green'
                start_time = time.time()
                while True:
                    if time.time() - start_time > 1:
                        break
            elif place_index < f:
                carrying_object = False
                object_color[0] = 'cyan'


            # Update object position
            if carrying_object:
                object_pos = joint_positions[-1]

        # Plot object
        if object_pos is not None:
            ax.scatter(*object_pos, color=object_color[0], s=100, label='Object')

        # Plot static Cartesian trajectory
        ax.plot(ee_x, ee_y, ee_z, color='blue', linestyle='--', label='Cartesian Trajectory')

        # Plot robot at current frame
        plot_robot(joint_positions, rotation_axes, ax=ax, obstacles=obstacles, title=f"Step {f+1}")

        ax.legend()
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([0, 1.5])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

    # Increase total frames to hold pick and place frames longer
    total_frames = len(joint_trajectory)
    ani = FuncAnimation(fig, update, frames=total_frames, interval=100, blit=False)
    plt.tight_layout()
    plt.show()
