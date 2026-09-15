#!/usr/bin/env python3
"""
Generate pose_pool_30_trials_world3.json for 3-Room World 3 map.
Contains 30 valid full-mission episodes starting in open Zone 1 (y=-8.5) and ending in Zone 5 (y=8.0).
"""

import os
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GAZEBO_SIM_DIR = os.path.dirname(SCRIPT_DIR)
SRC_DIR = os.path.dirname(GAZEBO_SIM_DIR)
EVAL_RESULTS_DIR = os.path.join(SRC_DIR, "online_causal_tuner", "evaluation_results")
os.makedirs(EVAL_RESULTS_DIR, exist_ok=True)

def generate_pose_pool():
    start_x_list = [-1.0, -0.5, 0.0, 0.5, 1.0]
    goal_x_list = [-1.0, -0.5, 0.0, 0.5, 1.0]

    episodes = []
    count = 1

    for s_x in start_x_list:
        for g_x in goal_x_list:
            episodes.append({
                "episode_id": count,
                "start_pose": {"x": float(s_x), "y": -8.50, "yaw": 1.5708},
                "goal_pose": {"x": float(g_x), "y": 8.00, "yaw": 1.5708}
            })
            count += 1

    extra_starts = [-0.75, -0.25, 0.0, 0.25, 0.75]
    extra_goals = [0.75, 0.25, 0.0, -0.25, -0.75]
    for s_x, g_x in zip(extra_starts, extra_goals):
        episodes.append({
            "episode_id": count,
            "start_pose": {"x": float(s_x), "y": -8.60, "yaw": 1.5708},
            "goal_pose": {"x": float(g_x), "y": 8.20, "yaw": 1.5708}
        })
        count += 1

    out_path = os.path.join(EVAL_RESULTS_DIR, "pose_pool_30_trials_world3.json")
    with open(out_path, "w") as f:
        json.dump(episodes, f, indent=2)
    print(f"Successfully generated {len(episodes)} episodes in {out_path}")

if __name__ == "__main__":
    generate_pose_pool()
