#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import String, Bool

import cv2
import numpy as np
import torch

from tf2_ros import Buffer, TransformListener

from lerobot.policies.act.modeling_act import ACTPolicy, ACTTemporalEnsembler
from lerobot.processor.pipeline import DataProcessorPipeline

# Import data collection utilities to match tuning and logic exactly
from lerobot_integration.data_collector import (
    generate_random_closed_shape,
    robot_to_pixel,
    pixel_to_robot,
    Z_CANVAS,
    Z_TOLERANCE,
    CANVAS_SIZE
)
import time


class PolicyExecutor(Node):
    def __init__(self):
        super().__init__('policy_executor')

        self.declare_parameter('ensemble_coeff', 0.05)
        self.declare_parameter('timer_frequency', 30.0)
        self.declare_parameter('goal_complete_freq', 50.0)
        
        
        # Load policy from the extracted zip folder
        self.declare_parameter('model_path', '/home/robot/vla_ws/src/lerobot_integration/trained_policy/ACT/v4')
        model_path = self.get_parameter('model_path').value
        
        self.get_logger().info(f"Loading ACT policy from {model_path}...")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        try:
            self.policy = ACTPolicy.from_pretrained(model_path)
            self.policy.to(self.device)
            self.policy.eval()
            self.policy.reset()
            
            self.preprocessor = DataProcessorPipeline.from_pretrained(
                model_path,
                config_filename="policy_preprocessor.json",
                overrides={"device_processor": {"device": str(self.device)}}
            )
            self.postprocessor = DataProcessorPipeline.from_pretrained(
                model_path,
                config_filename="policy_postprocessor.json",
            )
            
            # Enable Temporal Ensembling (CRITICAL for closed-loop ACT tracking)
            # This continuously averages overlapping chunks to prevent phase shift / spiraling.
            self.policy.config.temporal_ensemble_coeff = self.get_parameter('ensemble_coeff').value
            self.policy.temporal_ensembler = ACTTemporalEnsembler(
                self.policy.config.temporal_ensemble_coeff, 
                self.policy.config.chunk_size
            )
                
            self.get_logger().info(f"Policy + processors loaded successfully on {self.device}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load policy: {e}")
            import traceback
            traceback.print_exc()
            self.policy = None
            self.preprocessor = None
            self.postprocessor = None

        # TF listener to track end-effector position for drawing progress
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # UI and Trajectory States
        self.progress_trail = []       # [(px, py), ...]
        self.progress_points_3d = []   # [(rx, ry, rz), ...]
        self.goal_canvas_display = None

        # tuning parameters
        self.timer_frequency = self.get_parameter('timer_frequency').value
        self.goal_complete_freq = self.get_parameter('goal_complete_freq').value
        
        
        # Subscribers and Publishers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        self.marker_pub = self.create_publisher(
            Marker,
            'draw',
            10
        )

        self.debug_info = self.create_publisher(
            String,
            '/debug_info',
            10
        )
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_5']
        self.current_joints = None
        self.all_joint_names = []
        self.all_current_joints = []
        
        # Generate the goal target and setup markers
        self.generate_goal()
        
        # Create IK Client for high-speed Cartesian-to-Joint conversion
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Publisher to stream joint commands directly to the hardware controller
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # Timer for control loop (30 Hz for action execution matching training data)
        self.control_timer = self.create_timer(1.0 / self.timer_frequency, self.control_step)
        
        # State flag to prevent overlapping IK calls
        self.ik_pending = False

    def generate_goal(self):
        """Generate a random closed shape and set up the RViz goal marker."""
        self.get_logger().info("Generating random shape for evaluation...")
        self.goal_canvas_display, goal_pixel_coords = generate_random_closed_shape()
        
        # Precompute the goal marker (Blue line on the canvas)
        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = 'world'
        self.goal_marker.ns = 'goal_shape'
        self.goal_marker.type = Marker.LINE_STRIP
        self.goal_marker.action = Marker.ADD
        self.goal_marker.id = 0
        self.goal_marker.scale.x = 0.003 # 3mm line
        self.goal_marker.color.a = 0.8
        self.goal_marker.color.r = 0.0
        self.goal_marker.color.g = 0.5
        self.goal_marker.color.b = 1.0
        
        for px, py in goal_pixel_coords:
            rx, ry, rz = pixel_to_robot(px, py, Z_CANVAS)
            p = Point()
            p.x, p.y, p.z = rx, ry, rz
            self.goal_marker.points.append(p)
            
        self.get_logger().info("Goal shape generated successfully.")

    def joint_callback(self, msg):
        # Save all joints for MoveIt IK Seed
        self.all_joint_names = list(msg.name)
        self.all_current_joints = list(msg.position)
        
        # Extract the 4 specific joints for Policy State Vector
        positions = []
        try:
            for name in self.joint_names:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joints = positions
        except ValueError:
            pass

    def get_ee_position(self):
        """Look up EE position from TF. Returns (x, y, z) or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'ee_link', rclpy.time.Time()
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
        except Exception:
            return None

    def render_progress_canvas(self, cursor_px, cursor_py, pen_down):
        """Match data collection: white canvas, black trail, green/red cursor."""
        canvas = np.ones((CANVAS_SIZE, CANVAS_SIZE, 3), dtype=np.uint8) * 255

        # Draw accumulated trail in black
        if len(self.progress_trail) > 1:
            for i in range(len(self.progress_trail) - 1):
                cv2.line(
                    canvas,
                    self.progress_trail[i],
                    self.progress_trail[i + 1],
                    (0, 0, 0), 2
                )

        # Draw cursor (green=down, red=up)
        if pen_down:
            color = (0, 200, 0)
            radius = 12
        else:
            color = (0, 0, 200)
            radius = 6

        cv2.circle(canvas, (cursor_px, cursor_py), radius, color, -1)
        return canvas

    def prepare_image_tensor(self, img_bgr):
        """Convert a BGR image to the format expected by the dataset: RGB, resized 224x224, and scaled to [0, 1]."""
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        # Thicken lines and use area interpolation to maintain line continuity (matching dataset conversion)
        kernel = np.ones((5, 5), np.uint8)
        img_thick = cv2.erode(img_rgb, kernel, iterations=1)
        img_resized = cv2.resize(img_thick, (224, 224), interpolation=cv2.INTER_AREA)
        
        tensor = torch.from_numpy(img_resized).permute(2, 0, 1).float() / 255.0
        return tensor

    def publish_markers(self):
        """Publish RViz markers for the goal shape and the drawn progress."""
        # Publish static goal
        self.goal_marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.goal_marker)
        
        # Publish dynamic progress trail (matching commander style)
        prog_marker = Marker()
        prog_marker.header.frame_id = 'world'
        prog_marker.ns = 'drawing'
        prog_marker.header.stamp = self.get_clock().now().to_msg()
        prog_marker.type = Marker.LINE_STRIP
        prog_marker.action = Marker.ADD
        prog_marker.id = 0
        prog_marker.scale.x = 0.005 # 5mm line
        prog_marker.color.a = 1.0
        prog_marker.color.r = 1.0 # Red trail like commander
        prog_marker.color.g = 0.0
        prog_marker.color.b = 0.0
        
        for x, y, z in self.progress_points_3d:
            p = Point()
            p.x, p.y, p.z = x, y, z
            prog_marker.points.append(p)
            
        self.marker_pub.publish(prog_marker)

    def control_step(self):
        if self.ik_pending:
            return # Don't start another control step if IK is still computing
            
        if self.policy is None or self.current_joints is None or self.goal_canvas_display is None:
            return
        if self.preprocessor is None or self.postprocessor is None:
            return
        
        # 1. Lookup End-Effector and calculate compliance variables
        ee_pos = self.get_ee_position()
        if ee_pos is None:
            return
            
        ee_x, ee_y, ee_z, qx, qy, qz, qw = ee_pos
        pen_down = abs(ee_z - Z_CANVAS) < Z_TOLERANCE
        cursor_px, cursor_py = robot_to_pixel(ee_x, ee_y)
        
        if pen_down:
            self.progress_trail.append((cursor_px, cursor_py))
            self.progress_points_3d.append((ee_x, ee_y, ee_z))
            
        # 2. Build 8-dimensional State Vector: [j1, j2, j3, j5, ee_x, ee_y, ee_z, pen_down]
        state_vec = self.current_joints + [ee_x, ee_y, ee_z, 1.0 if pen_down else 0.0]
        state_tensor = torch.tensor(state_vec, dtype=torch.float32)
            
        # 3. Generate visual inputs matching data collection
        progress_img = self.render_progress_canvas(cursor_px, cursor_py, pen_down)
        goal_bgr = cv2.cvtColor(self.goal_canvas_display, cv2.COLOR_GRAY2BGR)
        
        goal_tensor = self.prepare_image_tensor(goal_bgr)
        prog_tensor = self.prepare_image_tensor(progress_img)
        
        # 4. Query Policy (Runs at 30Hz closed-loop with temporal ensembling)
        obs = {
            "observation.images.goal": goal_tensor,
            "observation.images.progress": prog_tensor,
            "observation.state": state_tensor
        }
        
        processed_obs = self.preprocessor(obs)
        
        with torch.no_grad():
            # Use Temporal Ensembling: safe with absolute coords, smooths target positions
            action = self.policy.select_action(processed_obs)
            
        action_out = self.postprocessor({"action": action})
        action_abs = action_out["action"].squeeze(0).cpu().numpy()  # [3] = (x, y, z) absolute
        
        # 5. Convert Absolute Cartesian Position to Joint Angles via MoveIt IK
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = float(action_abs[0])
        target_pose.pose.position.y = float(action_abs[1])
        target_pose.pose.position.z = float(action_abs[2])
        
        # Force perfect vertical orientation to guarantee AR4 IK solver success
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.pose_stamped = target_pose
        
        # Seed the IK solver with ALL current joints to prevent "empty JointState message" error
        req.ik_request.robot_state.joint_state.name = self.all_joint_names
        req.ik_request.robot_state.joint_state.position = self.all_current_joints
        
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 20000000 # 20ms timeout for IK
        
        # Call IK asynchronously
        if self.ik_client.wait_for_service(timeout_sec=0.1):
            self.ik_pending = True
            future = self.ik_client.call_async(req)
            future.add_done_callback(
                lambda fut, progress_img=progress_img, goal_bgr=goal_bgr: self.ik_callback(fut, progress_img, goal_bgr)
            )
        else:
            self.get_logger().warn("IK Service not available!")
            return

    def ik_callback(self, future, progress_img, goal_bgr):
        self.ik_pending = False
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            return
            
        if res.error_code.val != 1: # 1 is SUCCESS in MoveIt
            self.get_logger().warn("IK Failed for target position!")
            return
        
        # Extract target joints
        target_joints = []
        for name in self.joint_names:
            if name in res.solution.joint_state.name:
                idx = res.solution.joint_state.name.index(name)
                target_joints.append(res.solution.joint_state.position[idx])
            else:
                self.get_logger().error(f"Joint {name} not found in IK solution")
                return
        action_joints = target_joints
        
        # 6. Execute action using a stable velocity-profile for the trajectory controller
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = action_joints
        
        # Calculate target velocities to prevent the controller from stopping at the waypoint
        dt = 1.0 / self.goal_complete_freq
        current_joints_ordered = []
        for name in self.joint_names:
            idx = self.all_joint_names.index(name)
            current_joints_ordered.append(self.all_current_joints[idx])
            
        point.velocities = [(t - c) / dt for t, c in zip(action_joints, current_joints_ordered)]
        
        # Exact 33ms interpolation to match the 30Hz control loop
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(dt * 1e9)
        
        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)
        
        # 6. Update visual feedback (OpenCV and RViz)
        prog_display = cv2.resize(progress_img, (500, 500))
        goal_resized = cv2.resize(goal_bgr, (500, 500))
        combined = np.hstack([goal_resized, prog_display])
        cv2.imshow("Policy Evaluation: Goal | Progress", combined)
        cv2.waitKey(1)
        
        self.publish_markers()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

