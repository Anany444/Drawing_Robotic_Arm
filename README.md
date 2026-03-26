# Drawing_Robotic_Arm
An AR4 based Robotic Arm in Gazebo simulation which takes user drawn input image and generate a trajectory using MoveIt2 C++ API which is followed by the end effector tool and visualised in Rviz.

## System Architecture

The system implements a modular pipeline integrating user input, motion planning, control, simulation and visualization:

### 1. Input Layer (OpenCV)
- A Python OpenCV script captures 2D mouse strokes.
- Coordinates are normalized and stored as `(x, y, z)` waypoints in `coords.txt`, where `z` defines the drawing plane height.

### 2. Planning Layer (MoveIt 2)
- A C++ node reads waypoints and generates a Cartesian trajectory using the MoveIt2 C++ planning interface .

### 3. Control & Simulation (ros2_control + Gazebo)
- The trajectory is executed via `ros2_control` joint trajectory controllers.
- The `gz_ros2_control` plugin interfaces controllers with the Gazebo-simulated AR4 robot.

### 4. Visualization (RViz)
- RViz visualizes the drawing using markers for end effector trajectory at the drawing plane height.



## Repository Structure

```text
src/
├── bringup/                  # Launch files and runtime configs
│   ├── launch/
│   └── config/
├── commander/                # C++ MoveIt2 API node
│   └── src/
├── cv/                       # OpenCV drawing input tool
│   └── src/
├── description/              # Gazebo world assets
│   └── worlds/
├── annin_ar4_description/    # URDF/Xacro + meshes
│   ├── urdf/
│   ├── meshes/
│   └── config/
└── moveit_setup/             # MoveIt2 configuration
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

# Source Ros2
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash

# Install Python dependencies  
pip install opencv-python numpy

```
## Usage
### 1. Launch everything
```bash
# Source workspace
source ~/drawing_arm_ws/install/setup.bash

#Launch full simulation (Gazebo + Rviz + MoveGroup + Controllers)
ros2 launch bringup final.launch.py
```
### 2. Drawing input
```bash
# In a new terminal 
cd ~/drawing_arm_ws/src/cv/src

# Run the drawing Python script
python3 drawing_input.py  #draw, save with s and quit with q
```
### 3. Execute Trajectory
```bash
# In another terminal
cd ~/drawing_arm_ws/src/cv/src
source ~/drawing_arm_ws/install/setup.bash

# Run MoveIt2 commander moveit cpp api node
ros2 run commander commander   #Followed trajectory will be visualised in Rviz
```



