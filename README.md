# Drawing_Robotic_Arm

An AR4 Robotic Arm in a Gazebo simulation. This project allows you to generate and execute drawing trajectories in two different ways:
1. **Classical Planning:** Takes a user-drawn input image and generates a trajectory using MoveIt2.
2. **ACT Policy:** Uses an Action Chunking with Transformers (ACT) model trained on synthetic, procedurally generated trajectory data to generate and draw shapes autonomously.


## Features
- **OpenCV Input:** A simple drawing tool that captures mouse strokes and converts them into Cartesian coordinates.
- **MoveIt2 Integration:** C++ node for standard trajectory planning and execution.
- **AI Policy Integration:** Uses Hugging Face's LeRobot library to run an ACT policy with temporal ensembling for smooth control.
- **Simulation:** Full Gazebo physics simulation and RViz visualization.


## Video Demo
https://github.com/user-attachments/assets/c285da72-82ae-4704-a958-bd43ca4de112


## 📊 ACT Policy Results

Here is a comparison of the randomly generated target shape (on the left in each image) versus the ACT Policy's executed trajectory (On the right in each image).

<p align="center">
  <img src="https://github.com/user-attachments/assets/e706d3e7-8324-461a-89d7-afac4d9cbc14" width="32%" alt="Result 1" />
  <img src="https://github.com/user-attachments/assets/a3dd4887-b422-4d0e-8d6e-90dd87ef7162" width="32%" alt="Result 2" />
  <img src="https://github.com/user-attachments/assets/1b031279-cf33-4730-af73-1cdccb3ae15d" width="32%" alt="Result 3" />
  <br>
  <img src="https://github.com/user-attachments/assets/2177f963-cee0-4d5c-89f9-03d3007faa14" width="32%" alt="Result 4" />
  <img src="https://github.com/user-attachments/assets/9c8b7402-6f5c-46cd-a9dc-81250757cbb4" width="32%" alt="Result 5" />
  <img src="https://github.com/user-attachments/assets/ed384339-ba6a-403e-8d6e-702aefed16f3" width="32%" alt="Result 6" />
</p>


## System Architecture


The project features a modular pipeline that handles both classical and policy-based execution:


### 1. Input Generation
- **User Input (Classical):** A Python OpenCV script captures 2D mouse strokes. Coordinates are normalized and stored as (x, y, z) waypoints in `coords.txt`.
- **Procedural Generation (Policy):** A procedural generation function creates random closed shapes and outputs them as a visual target (canvas image) for the model.

### 2. Planning and Policy Execution
- **MoveIt 2 (Classical):** A C++ node reads the `coords.txt` waypoints and generates a Cartesian trajectory using the MoveIt2 C++ planning interface.
- **ACT Policy:** An ACT (Action Chunking with Transformers) policy takes three inputs: a target goal image, a live progress image, and an 8-dimensional robot state vector (joints + end-effector position). It outputs a sequence of absolute Cartesian (X, Y, Z) coordinates. Temporal ensembling averages these overlapping action chunks at 30Hz, and the resulting target is converted to joint angles via MoveIt Inverse Kinematics service for smooth, continuous movement.

### 3. Control & Simulation
- Trajectories from both modes are executed via `ros2_control` joint trajectory controllers.
- The `gz_ros2_control` plugin interfaces the controllers with the Gazebo-simulated AR4 robot.

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
└── lerobot_integration/     # Policy execution, data collection and data conversion nodes
|   ├── lerobot_integration/
|   └── trained_policy/
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

### 1. Setup Workspace and ROS Dependencies
```bash
# Create workspace
mkdir -p ~/drawing_arm_ws/src
cd ~/drawing_arm_ws/src

# Clone repository
git clone https://github.com/Anany444/Drawing_Robotic_Arm.git
cd ~/drawing_arm_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

### 2. Python Dependencies
```bash
pip install opencv-python numpy torch huggingface_hub
```

**Install Hugging Face LeRobot (Needed for the ACT policy):**
```bash
cd ~/drawing_arm_ws/src/lerobot_integration
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e .
```
### 3. Dataset and Model Weights
Refer to policy.md file in lerobot_integration/trained_policy folder


## Usage
### Step 1: Launch the Simulation
Always start by launching Gazebo, MoveGroup, and the controllers:
```bash
cd ~/drawing_arm_ws
source ~/drawing_arm_ws/install/setup.bash
ros2 launch bringup final.launch.py
```
### Step 2: Choose Execution Mode

#### Option A: User Input (MoveIt2 C++ API)
Draw a shape manually and have the C++ node generate the trajectory.

1. **Open the Drawing Tool (New Terminal):**
   ```bash
   cd ~/drawing_arm_ws/src/cv/src
   python3 drawing_input.py
   # Draw your shape, press 's' to save, and 'q' to quit.
   ```
2. **Execute (New Terminal):**
   ```bash
   cd ~/drawing_arm_ws/src/cv/src
   source ~/drawing_arm_ws/install/setup.bash
   ros2 run commander commander
   ```

#### Option B: ACT Policy
Have the trained model randomly generate and draw a shape on its own.

1. **Run the Policy Executor (New Terminal):**
   ```bash
   source ~/drawing_arm_ws/install/setup.bash
   ros2 run lerobot_integration policy_executor
   ```
   *This will generate a random shape, show the target line in RViz, and run the ACT policy to draw it.*
```


