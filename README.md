# 6R Motion Control – Cartesian Motion of a 6-DOF Robot Arm

## Overview

This project implements Cartesian motion control for a 6-axis robotic arm with 6 degrees of freedom (DOF). The system supports smooth point-to-point movements in 3D space with full orientation control. Motion planning includes optional obstacle avoidance and a basic pick-and-place routine.


## Objectives

1. Define a 6-joint robotic arm with joint limits and forward/inverse kinematics.
2. Accept Cartesian targets (x, y, z, roll, pitch, yaw) from the user or generate them randomly.
3. Plan smooth Cartesian trajectories using linear interpolation.
4. Execute and visualize the robot's motion using Python.
5. Obstacle avoidance with spherical objects in the workspace.
6. Pick-and-place execution with grasp and release logic.
   
## Features

- Forward and inverse kinematics using DH parameters
- Linear interpolation for smooth Cartesian motion
- User-defined or randomly generated poses
- Optional spherical obstacles
- Pick-and-place demonstration
- 3D visualization of the robot and its trajectory
- Joint space trajectory plotting


## Repository Structure
```
6R_motion_control/
├── .gitignore
├── main.py                   # Point-to-point Cartesian motion with user/CUI
├── pick_and_place.py         # Pick-and-place sequence with optional obstacles
├── robot/                    # Robot modules and motion logic
│   ├── __init__.py
│   ├── kinematics.py         # Forward/inverse kinematics
│   ├── obstacle_avoidance.py # Obstacle-aware planning logic
│   ├── path_planning.py      # Cartesian trajectory generation
│   ├── robot_description.py  # DH parameters and joint limits
│   ├── utils.py              # Helper functions (pose generation, workspace checks)
│   └── visualization.py      # 3D visualization of the robot and motion
└── README.md
```

## Installation

Requires Python 3.8 or later.

Install dependencies with:

```bash
pip install numpy matplotlib
```


## How to Run

### 1. Point-to-Point Cartesian Motion


Run the main script:

```bash
python main.py
```

- Choose between custom or random start/end poses

- Choose whether to include obstacles

The script checks kinematic feasibility, plans the trajectory, and visualizes it.

### 2. Pick-and-Place Execution

Run the full scenario:

```bash
python pick_and_place.py
```
- Choose between custom or random pick/place poses

- Choose whether to include obstacles

The script performs:

- Home → Pick → Place → Home motion

- Optional obstacle inclusion

- Animated 3D visualization and joint angle plots

## Design Notes:

- Kinematics are implemented analytically for 6R arms using DH parameters.

- Trajectory generation is linear in Cartesian space with fixed step size.

- Obstacles are represented as spheres, randomly placed between segments.

- The project uses Matplotlib for visualization.



## Author

### Ege Cansev
