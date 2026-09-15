#!/usr/bin/env python3
"""
World 3 & Pose Pool Checker.
Verifies carry envelope feasibility on start/goal poses and carry adoption fraction on free map cells.
Usage:
    python3 check_world.py <map.pgm> <map.yaml> --poses <pose_pool.json>
"""

import os
import sys
import math
import json
import argparse
import yaml
import numpy as np

# TIAGo carry envelope geometry constants
CARRY_ENVELOPE_WIDTH = 1.48
CARRY_CLEARANCE_REQUIRED = 0.847  # Clearance radius for carry adoption (inscribed/margin)

def load_pgm_map(pgm_path):
    with open(pgm_path, 'rb') as f:
        header = f.readline().decode('ascii')
        if not header.startswith('P5'):
            raise ValueError("Invalid PGM header format")
        line = f.readline().decode('ascii')
        while line.startswith('#'):
            line = f.readline().decode('ascii')
        width, height = map(int, line.split())
        max_val = int(f.readline().decode('ascii'))
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape((height, width))
    return data, width, height

def compute_distance_transform(grid_occupancy, resolution):
    """Compute Euclidean distance to nearest occupied cell in metres."""
    from scipy.ndimage import distance_transform_edt
    # Free cells = 1, occupied cells = 0
    free_mask = (grid_occupancy == 255)
    dist_grid = distance_transform_edt(free_mask) * resolution
    return dist_grid

def check_world(pgm_path, yaml_path, poses_path):
    if not os.path.exists(pgm_path):
        # Resolve path relative to maps dir if needed
        pgm_path = os.path.join("/home/forough/phd_projects/online_tuner/src/gazebo_simulation/maps", os.path.basename(pgm_path))
    if not os.path.exists(yaml_path):
        yaml_path = os.path.join("/home/forough/phd_projects/online_tuner/src/gazebo_simulation/maps", os.path.basename(yaml_path))
    if not os.path.exists(poses_path):
        poses_path = os.path.join("/home/forough/phd_projects/online_tuner/src/online_causal_tuner/evaluation_results", os.path.basename(poses_path))

    with open(yaml_path, 'r') as f:
        map_meta = yaml.safe_load(f)

    resolution = map_meta['resolution']
    origin_x, origin_y, _ = map_meta['origin']

    grid_data, width, height = load_pgm_map(pgm_path)
    # Rebuilt grid: PGM bottom row = min_y
    grid_flipped = np.flipud(grid_data)

    dist_grid = compute_distance_transform(grid_flipped, resolution)

    free_cells = (grid_flipped == 255)
    total_free = np.sum(free_cells)
    carry_adoption_cells = np.sum(free_cells & (dist_grid >= CARRY_CLEARANCE_REQUIRED))
    adoption_fraction = (carry_adoption_cells / total_free) * 100.0

    print(f"\n========================================================")
    print(f" WORLD 3 MAP VERIFICATION REPORT")
    print(f"========================================================")
    print(f" Map: {os.path.basename(yaml_path)} ({width}x{height} @ {resolution} m/cell)")
    print(f" Total Free Cells: {total_free}")
    print(f" Carry Adoption Clearance (>= {CARRY_CLEARANCE_REQUIRED:.3f} m): {carry_adoption_cells} cells ({adoption_fraction:.1f}%)")

    with open(poses_path, 'r') as f:
        episodes = json.load(f)

    print(f"\n Checking {len(episodes)} episodes ({len(episodes)*2} poses)...")

    all_poses_valid = True
    for ep in episodes:
        ep_id = ep["episode_id"]
        for ptype, pose in [("start", ep["start_pose"]), ("goal", ep["goal_pose"])]:
            px, py = pose["x"], pose["y"]
            col = int(round((px - origin_x) / resolution))
            row = int(round((py - origin_y) / resolution))

            if 0 <= col < width and 0 <= row < height:
                clearance = dist_grid[row, col]
                fits = clearance >= CARRY_CLEARANCE_REQUIRED
                if not fits:
                    all_poses_valid = False
                    print(f"  [FAIL] Episode {ep_id} {ptype} pose ({px:.2f}, {py:.2f}): clearance={clearance:.3f} m < {CARRY_CLEARANCE_REQUIRED:.3f} m")
            else:
                all_poses_valid = False
                print(f"  [FAIL] Episode {ep_id} {ptype} pose ({px:.2f}, {py:.2f}): OUTSIDE MAP")

    print(f"--------------------------------------------------------")
    print(f" All start/goal poses carry fits: {all_poses_valid}")
    print(f" Carry adoption clearance met on >= 25% free cells: {adoption_fraction >= 25.0} ({adoption_fraction:.1f}%)")
    print(f"========================================================\n")

    if all_poses_valid and adoption_fraction >= 25.0:
        print(">>> VERIFICATION PASSED ✓ <<<")
        sys.exit(0)
    else:
        print(">>> VERIFICATION FAILED ✗ <<<")
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check World 3 map and pose pool")
    parser.add_argument("pgm", type=str, help="Path to pgm map")
    parser.add_argument("yaml", type=str, help="Path to yaml map")
    parser.add_argument("--poses", type=str, required=True, help="Path to pose pool json")
    args = parser.parse_args()

    check_world(args.pgm, args.yaml, args.poses)
