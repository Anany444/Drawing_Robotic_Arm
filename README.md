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
│   ├── launch/
│   └── config/
├── commander/                # C++ MoveIt 2 API node
│   └── src/
├── cv/                       # OpenCV drawing input tool
│   └── src/
├── description/              # Gazebo world assets
│   └── worlds/
├── annin_ar4_description/    # URDF/Xacro + meshes
│   ├── urdf/
│   ├── meshes/
│   └── config/
└── moveit_setup/             # MoveIt 2 configuration
    ├── launch/
    └── config/
```

## Installation

```bash
# Create workspace
mkdir -p ~/drawing_arm_ws/src
cd ~/drawing_arm_ws/src

# Clone repository
git clone https://github.com/Anany444/Drawing_Robotic_Arm.git

# Move to workspace
cd ~/drawing_arm_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash

# Install Python dependencies  
pip install opencv-python numpy

```

write detailed struct
install
usage
sys arch
