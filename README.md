# Drawing_Robotic_Arm
An AR4 based Robotic Arm in gazebo simulation capable of taking drawn input image from user which is used to generate a trajectory using moveit2 c++ api which is followed by the end effector tool and visualised in rviz.

## Overview
This repository implements a simple drawing pipeline:

1. A Python OpenCV tool captures 2D mouse strokes.
2. The sampled screen coordinates are normalized and written to a `coords.txt` text file as `(x, y, z)` triplets where z represents canvas height.
3. A C++ MoveIt api reads those coordinates and generates a Cartesian trajectory through them.
4. The robot model is spawned in Gazebo Sim and controlled using `ros2_control` via gz_ros2_control plugin.
5. RViz is used for visualizing the end effector tool trajectory at canvas height.


## Repository Structure

```text
src/
├── bringup/                  # Launch files and runtime configs
├── commander/                # C++ MoveIt api commander node
├── cv/                       # Python drawing input utility
├── description/              # Gazebo world assets
├── annin_ar4_description/    # Urdf files, meshes
└── moveit_setup/             # MoveIt 2 configuration and launch files
```
