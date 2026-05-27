#!/usr/bin/env python3
"""
Enhanced data collector for LeRobot ACT policy training.

Improvements over v1:
- TF-based EE position tracking (x, y, z)
- Pen-up/pen-down paradigm based on EE Z proximity to canvas
- Progress canvas with: black trail (pen down) + colored cursor (green=down, red=up)
- Augmented state vector: [j1, j2, j3, j5, ee_x, ee_y, ee_z, pen_down]
- Random non-self-intersecting closed shapes via Fourier radius modulation
- Approach/retract Z movements in trajectory for pen-up/pen-down training data
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import cv2
import numpy as np
import os
import subprocess
import time
import json
import threading


# ── Constants ──────────────────────────────────────────────────────────────────
Z_CANVAS = 0.200       # Z height of the virtual drawing surface
Z_ABOVE  = 0.250       # Z height for pen-up (above canvas)
Z_TOLERANCE = 0.015    # ± tolerance for pen-down detection

CANVAS_SIZE = 1000     # pixel dimensions of the canvas
# Coordinate transform: pixel ↔ robot
# robot_x = (pixel_x - 500) / 3500
# robot_y = -(pixel_y / 3500 + 0.25)
SCALE  = 3500.0
OFFSET_X = 500.0
OFFSET_Y = 0.25

# Shape generation bounds (pixel space)
# Min radius ~220 gives area ≈ π·220² ≈ 152k px² (shapes with Fourier
# modulation expand well beyond base radius, easily exceeding 250k = 1/4 canvas)
SHAPE_CENTER_MIN = 450
SHAPE_CENTER_MAX = 550
SHAPE_RADIUS_MIN = 220
SHAPE_RADIUS_MAX = 350


def pixel_to_robot(px, py, z=Z_CANVAS):
    """Convert pixel coordinates to robot workspace coordinates."""
    rx = (px - OFFSET_X) / SCALE
    ry = -(py / SCALE + OFFSET_Y)
    return rx, ry, z


def robot_to_pixel(rx, ry):
    """Convert robot workspace coordinates to pixel coordinates."""
    px = int(rx * SCALE + OFFSET_X)
    py = int((-ry - OFFSET_Y) * SCALE)
    return px, py


def generate_random_closed_shape():
    """
    Generate a random non-self-intersecting closed shape using Fourier
    radius modulation in polar coordinates.

    The shape is defined as r(θ) = base_radius + Σ aₖ·sin(kθ + φₖ)
    where angles are sampled uniformly in [0, 2π). Sorting by angle
    and keeping all radii positive guarantees a simple (non-crossing) curve.

    Shapes with area < 25% of the canvas are rejected and regenerated.

    Returns:
        goal_canvas: 1000x1000 grayscale image with the shape drawn in black
        pixel_coords: list of [px, py] points tracing the shape (closed)
    """
    MIN_AREA_FRACTION = 0.25
    min_area = MIN_AREA_FRACTION * CANVAS_SIZE * CANVAS_SIZE

    while True:
        # Random center
        cx = np.random.randint(SHAPE_CENTER_MIN, SHAPE_CENTER_MAX)
        cy = np.random.randint(SHAPE_CENTER_MIN, SHAPE_CENTER_MAX)

        # Number of sample points along the curve (more = smoother)
        num_points = np.random.randint(60, 120)

        # Evenly spaced angles
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

        # Base radius + Fourier harmonics for organic variation
        base_radius = np.random.randint(SHAPE_RADIUS_MIN, SHAPE_RADIUS_MAX)
        num_harmonics = np.random.randint(2, 6)
        radii = np.ones(num_points) * base_radius

        for k in range(1, num_harmonics + 1):
            amplitude = np.random.uniform(-base_radius * 0.3, base_radius * 0.3)
            phase = np.random.uniform(0, 2 * np.pi)
            radii += amplitude * np.sin(k * angles + phase)

        # Ensure all radii are positive (no crossing through center)
        radii = np.clip(radii, 150, 420)

        # Convert polar to cartesian pixel coords
        pixel_coords = []
        for angle, r in zip(angles, radii):
            px = int(cx + r * np.cos(angle))
            py = int(cy + r * np.sin(angle))
            px = max(50, min(CANVAS_SIZE - 50, px))
            py = max(50, min(CANVAS_SIZE - 50, py))
            pixel_coords.append([px, py])

        # Close the shape (return to first point)
        pixel_coords.append(pixel_coords[0])

        # Check area — reject if too small
        contour = np.array(pixel_coords, dtype=np.int32)
        area = cv2.contourArea(contour)
        if area >= min_area:
            break
        # else: retry with new random params

    # Draw on canvas
    canvas = np.ones((CANVAS_SIZE, CANVAS_SIZE), dtype=np.uint8) * 255
    for i in range(len(pixel_coords) - 1):
        cv2.line(canvas, tuple(pixel_coords[i]), tuple(pixel_coords[i + 1]), 0, 2)

    return canvas, pixel_coords


def build_robot_trajectory(pixel_coords):
    """
    Convert pixel coordinates to a robot trajectory that includes
    approach (pen up), drawing (pen down), and retract (pen up) phases.

    Returns:
        List of [rx, ry, rz] waypoints.
    """
    waypoints = []

    first_rx, first_ry, _ = pixel_to_robot(pixel_coords[0][0], pixel_coords[0][1])

    # Phase 1: Approach — move to start point ABOVE canvas
    waypoints.append([first_rx, first_ry, Z_ABOVE])

    # Phase 2: Lower to canvas — pen down
    waypoints.append([first_rx, first_ry, Z_CANVAS])

    # Phase 3: Draw the shape at canvas height
    for pt in pixel_coords:
        rx, ry, rz = pixel_to_robot(pt[0], pt[1])
        waypoints.append([rx, ry, rz])

    # Phase 4: Retract — lift pen
    last_rx, last_ry = waypoints[-1][0], waypoints[-1][1]
    waypoints.append([last_rx, last_ry, Z_ABOVE])

    return waypoints


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # ── Subscriptions ──
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # ── TF listener for EE position ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ──
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_5']
        self.current_joints = None

        self.recording = False
        self.episode_data = []
        self.progress_trail = []   # accumulated (px, py) drawn when pen is down
        self.frame_idx = 0
        self.episode_dir = ""
        self.goal_canvas_display = None  # stored for main-thread visualization

        # ── 30 Hz recording timer ──
        self.record_timer = self.create_timer(1.0 / 30.0, self.record_step)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def joint_callback(self, msg):
        positions = []
        try:
            for name in self.joint_names:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joints = positions
        except ValueError:
            pass

    # ── EE lookup ──────────────────────────────────────────────────────────────

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
            )
        except Exception:
            return None

    # ── Progress canvas rendering ──────────────────────────────────────────────

    def render_progress_canvas(self, cursor_px, cursor_py, pen_down):
        """
        Render the progress canvas:
        - White background
        - Black trail where pen was down
        - Green circle = pen down cursor, Red circle = pen up cursor
        - Circle size: larger when pen is down (closer to surface)
        """
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

        # Draw cursor
        if pen_down:
            color = (0, 200, 0)   # green (BGR)
            radius = 12
        else:
            color = (0, 0, 200)   # red (BGR)
            radius = 6

        cv2.circle(canvas, (cursor_px, cursor_py), radius, color, -1)

        return canvas

    # ── Recording step ─────────────────────────────────────────────────────────

    def record_step(self):
        if not self.recording or self.current_joints is None:
            return

        # Get EE position from TF
        ee_pos = self.get_ee_position()
        if ee_pos is None:
            return

        ee_x, ee_y, ee_z = ee_pos

        # Determine pen state
        pen_down = abs(ee_z - Z_CANVAS) < Z_TOLERANCE

        # Convert EE XY to pixel coords for canvas
        cursor_px, cursor_py = robot_to_pixel(ee_x, ee_y)

        # Accumulate trail when pen is down
        if pen_down:
            self.progress_trail.append((cursor_px, cursor_py))

        # Render and save progress image
        progress_img = self.render_progress_canvas(cursor_px, cursor_py, pen_down)
        img_filename = f"progress_{self.frame_idx:06d}.png"
        img_path = os.path.join(self.episode_dir, "progress", img_filename)
        cv2.imwrite(img_path, progress_img)

        # Live visualization (runs on main thread via timer callback)
        prog_display = cv2.resize(progress_img, (500, 500))
        if self.goal_canvas_display is not None:
            # Show goal and progress side by side
            goal_bgr = cv2.cvtColor(self.goal_canvas_display, cv2.COLOR_GRAY2BGR)
            goal_resized = cv2.resize(goal_bgr, (500, 500))
            combined = np.hstack([goal_resized, prog_display])
            cv2.imshow("Data Collection: Goal | Progress", combined)
        else:
            cv2.imshow("Data Collection: Goal | Progress", prog_display)
        cv2.waitKey(1)

        # Build augmented state: [j1, j2, j3, j5, ee_x, ee_y, ee_z, pen_down]
        step_data = {
            "frame_idx": self.frame_idx,
            "timestamp": time.time(),
            "joints": self.current_joints,
            "ee_pos": [ee_x, ee_y, ee_z],
            "pen_down": 1.0 if pen_down else 0.0,
            "progress_img": img_filename,
        }
        self.episode_data.append(step_data)
        self.frame_idx += 1

    # ── Episode collection loop ────────────────────────────────────────────────

    def collect_episodes(self, num_episodes=500, output_dir='dataset'):
        os.makedirs(output_dir, exist_ok=True)

        for ep in range(num_episodes):
            self.get_logger().info(f"═══ Episode {ep}/{num_episodes} ═══")

            # 1. Generate random closed shape
            goal_canvas, pixel_coords = generate_random_closed_shape()

            # 2. Build robot trajectory (approach → draw → retract)
            robot_coords = build_robot_trajectory(pixel_coords)

            # 3. Setup episode directory
            self.episode_dir = os.path.join(output_dir, f"episode_{ep:04d}")
            os.makedirs(self.episode_dir, exist_ok=True)
            os.makedirs(os.path.join(self.episode_dir, "progress"), exist_ok=True)

            # 4. Save goal image (visualization handled in record_step on main thread)
            cv2.imwrite(os.path.join(self.episode_dir, "goal.png"), goal_canvas)
            cv2.imwrite("/home/robot/vla_ws/drawing.png", goal_canvas)
            self.goal_canvas_display = goal_canvas.copy()

            # 5. Write coords.txt for the commander
            with open("/home/robot/vla_ws/coords.txt", "w") as f:
                lines = [f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}" for p in robot_coords]
                f.write("\n".join(lines))

            time.sleep(1.0)

            # 6. Reset recording state
            self.episode_data = []
            self.frame_idx = 0
            self.progress_trail = []
            self.recording = True

            # 7. Execute trajectory via MoveIt commander
            self.get_logger().info("Running MoveIt commander...")
            process = subprocess.Popen(
                ["ros2", "run", "commander", "commander"],
                cwd="/home/robot/vla_ws"
            )
            process.wait()
            self.recording = False

            # 8. Save episode data
            data_path = os.path.join(self.episode_dir, "joint_data.json")
            with open(data_path, "w") as f:
                json.dump(self.episode_data, f)

            self.get_logger().info(
                f"Episode {ep} done: {len(self.episode_data)} frames, "
                f"{len(self.progress_trail)} trail points"
            )
            time.sleep(2.0)

        self.get_logger().info(f"All {num_episodes} episodes collected!")


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()

    thread = threading.Thread(
        target=node.collect_episodes,
        args=(500, '/home/robot/vla_ws/raw_dataset_v2')
    )
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
