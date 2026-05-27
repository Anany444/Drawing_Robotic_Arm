#!/usr/bin/env python3
"""
Dataset converter for LeRobot v3.0 format.

Converts raw episodes (from data_collector v2) into the chunked LeRobot format.

Features:
- observation.images.goal:     (3, 96, 96)  static target shape
- observation.images.progress: (3, 96, 96)  trail + cursor canvas (RGB)
- observation.state:           (8,)         [j1, j2, j3, j5, ee_x, ee_y, ee_z, pen_down]
- action:                      (4,)         [j1, j2, j3, j5] next joint positions
"""
import os
import json
import cv2
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def convert_dataset(raw_dir, output_repo_id, local_dir):
    """Converts raw recorded episodes into LeRobot v3.0 format."""

    features = {
        "observation.images.goal": {
            "dtype": "video",
            "shape": (3, 224, 224),
            "names": ["channels", "height", "width"],
        },
        "observation.images.progress": {
            "dtype": "video",
            "shape": (3, 224, 224),
            "names": ["channels", "height", "width"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (8,),
            "names": [
                "joint_1", "joint_2", "joint_3", "joint_5",
                "ee_x", "ee_y", "ee_z", "pen_down",
            ],
        },
        "action": {
            "dtype": "float32",
            "shape": (3,),
            "names": ["x", "y", "z"],
        },
    }

    print(f"Creating LeRobotDataset in {local_dir}")
    dataset = LeRobotDataset.create(
        repo_id=output_repo_id,
        fps=30,
        features=features,
        root=local_dir,
        vcodec="h264"
    )

    episodes = sorted([d for d in os.listdir(raw_dir) if d.startswith("episode_")])

    for ep in episodes:
        ep_dir = os.path.join(raw_dir, ep)
        print(f"Processing {ep}...")

        # ── Load goal image ──
        goal_img_path = os.path.join(ep_dir, "goal.png")
        goal_img = cv2.imread(goal_img_path)
        if goal_img is None:
            print(f"  Skipping {ep}: goal.png not found")
            continue

        # Convert grayscale goal to RGB if needed
        if len(goal_img.shape) == 2:
            goal_img = cv2.cvtColor(goal_img, cv2.COLOR_GRAY2RGB)
        else:
            goal_img = cv2.cvtColor(goal_img, cv2.COLOR_BGR2RGB)
            
        # Thicken the black lines before resizing to prevent dashed artifacts
        kernel = np.ones((5, 5), np.uint8)
        goal_img = cv2.erode(goal_img, kernel, iterations=1)
        
        goal_img = cv2.resize(goal_img, (224, 224), interpolation=cv2.INTER_AREA)
        goal_tensor = torch.from_numpy(goal_img).permute(2, 0, 1)  # [C, H, W]

        # ── Load joint data ──
        data_path = os.path.join(ep_dir, "joint_data.json")
        if not os.path.exists(data_path):
            print(f"  Skipping {ep}: joint_data.json not found")
            continue

        with open(data_path, "r") as f:
            ep_data = json.load(f)

        if len(ep_data) == 0:
            print(f"  Skipping {ep}: empty data")
            continue

        for i, step in enumerate(ep_data):
            # ── Build augmented state [j1..j5, ee_x, ee_y, ee_z, pen_down] ──
            joints = step["joints"]                    # [j1, j2, j3, j5]
            ee_pos = step.get("ee_pos", [0.0, 0.0, 0.0])  # [x, y, z]
            pen_down = step.get("pen_down", 0.0)

            import random
            # 1. Inject +/- 2mm of noise to the End Effector State
            noisy_ee_x = ee_pos[0] + random.uniform(-0.002, 0.002)
            noisy_ee_y = ee_pos[1] + random.uniform(-0.002, 0.002)
            
            # 2. Inject a tiny bit of noise to the Joint States (optional but recommended)
            noisy_joints = [j + random.uniform(-0.01, 0.01) for j in joints]

            state = np.array(
                noisy_joints + [noisy_ee_x, noisy_ee_y, ee_pos[2], pen_down],
                dtype=np.float32,
            )

            # ── Action = next step's Absolute Cartesian Coordinates [x, y, z] ──
            if i < len(ep_data) - 1:
                next_ee_pos = ep_data[i + 1].get("ee_pos", ee_pos)
                action = np.array([
                    next_ee_pos[0],
                    next_ee_pos[1],
                    next_ee_pos[2]
                ], dtype=np.float32)
            else:
                action = np.array([
                    ee_pos[0],
                    ee_pos[1],
                    ee_pos[2]
                ], dtype=np.float32)

            # ── Load progress image (now RGB with cursor) ──
            prog_img_path = os.path.join(ep_dir, "progress", step["progress_img"])
            if os.path.exists(prog_img_path):
                prog_img = cv2.imread(prog_img_path)
                prog_img = cv2.cvtColor(prog_img, cv2.COLOR_BGR2RGB)
                
                # Thicken lines and use area interpolation to maintain line continuity
                prog_img = cv2.erode(prog_img, kernel, iterations=1)
                prog_img = cv2.resize(prog_img, (224, 224), interpolation=cv2.INTER_AREA)
            else:
                prog_img = np.ones((224, 224, 3), dtype=np.uint8) * 255

            prog_tensor = torch.from_numpy(prog_img).permute(2, 0, 1)  # [C, H, W]

            # ── Add frame ──
            dataset.add_frame({
                "observation.images.goal": goal_tensor,
                "observation.images.progress": prog_tensor,
                "observation.state": torch.from_numpy(state),
                "action": torch.from_numpy(action),
                "task": "draw the shape",
            })

        dataset.save_episode()
        print(f"  Saved {ep} with {len(ep_data)} frames.")

    print("All episodes saved. Consolidating dataset...")
    try:
        dataset.consolidate()
        print("Dataset consolidated successfully.")
    except AttributeError:
        # v0.4.4 may auto-consolidate
        print("Dataset saved (consolidate not needed in this version).")


def main(args=None):
    raw_directory = "/home/robot/vla_ws/src/lerobot_integration/dataset/raw_dataset/v2"
    out_repo = "robot/drawing_dataset_v4"
    out_local = "/home/robot/vla_ws/src/lerobot_integration/dataset/lerobot_dataset/v4"
    convert_dataset(raw_directory, out_repo, out_local)


if __name__ == "__main__":
    main()
