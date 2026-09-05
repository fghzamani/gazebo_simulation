#!/usr/bin/env python3
"""
Generator for causal_benchmark_2 World, SDF, PGM, and YAML map files.
Follows the exact design specification in causal_benchmark_2_spec.md.
"""

import os
import sys

GAZEBO_SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.dirname(GAZEBO_SIM_DIR)
PAL_GAZEBO_WORLDS_DIR = os.path.join(SRC_DIR, "pal_gazebo_worlds", "worlds")

WORKSPACE_ROOT = os.path.dirname(SRC_DIR)

WORLDS_DIRS = [
    os.path.join(GAZEBO_SIM_DIR, "worlds"),
    PAL_GAZEBO_WORLDS_DIR,
    os.path.join(WORKSPACE_ROOT, "install", "gazebo_simulation", "share", "gazebo_simulation", "worlds"),
    os.path.join(WORKSPACE_ROOT, "install", "pal_gazebo_worlds", "share", "pal_gazebo_worlds", "worlds"),
    os.path.join(WORKSPACE_ROOT, "src", "install", "gazebo_simulation", "share", "gazebo_simulation", "worlds"),
    os.path.join(WORKSPACE_ROOT, "src", "install", "pal_gazebo_worlds", "share", "pal_gazebo_worlds", "worlds"),
]
MAPS_DIRS = [
    os.path.join(GAZEBO_SIM_DIR, "maps"),
    os.path.join(WORKSPACE_ROOT, "install", "gazebo_simulation", "share", "gazebo_simulation", "maps"),
    os.path.join(WORKSPACE_ROOT, "src", "install", "gazebo_simulation", "share", "gazebo_simulation", "maps"),
]

for d in WORLDS_DIRS:
    os.makedirs(d, exist_ok=True)
for d in MAPS_DIRS:
    os.makedirs(d, exist_ok=True)


def create_wall_box_sdf(name: str, x: float, y: float, z: float, dx: float, dy: float, dz: float = 2.0) -> str:
    """Generate SDF string for a static wall/obstacle box model."""
    return f"""
    <model name='{name}'>
      <static>1</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{dx:.4f} {dy:.4f} {dz:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{dx:.4f} {dy:.4f} {dz:.4f}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world_2():
    map_name = "causal_benchmark_2"
    
    # 2D Grid setup: 12m x 32m, resolution 0.05 m/px => 240 x 640 cells
    res = 0.05
    origin_x = -6.0
    origin_y = -16.0
    width = int(round(12.0 / res))   # 240
    height = int(round(32.0 / res))  # 640

    # Initialize grid with 255 (free space)
    grid = [[255 for _ in range(width)] for _ in range(height)]

    # List of box obstacles (name, x, y, z, dx, dy, dz)
    # Note: Outer walls are centered at +/-5.90m and +/-15.90m so their 0.20m thickness falls exactly inside [-6, 6] and [-16, 16]
    boxes = [
        # Outer perimeter walls
        ("outer_wall_west",  -5.90,   0.00, 1.00, 0.20, 32.00, 2.00),
        ("outer_wall_east",   5.90,   0.00, 1.00, 0.20, 32.00, 2.00),
        ("outer_wall_south",  0.00, -15.90, 1.00, 12.00,  0.20, 2.00),
        ("outer_wall_north",  0.00,  15.90, 1.00, 12.00,  0.20, 2.00),

        # Zone 1 Doorway at y = -11.00 (2.60m gap at x=0)
        ("z1_door_west",     -3.65, -11.00, 1.00, 4.70, 0.20, 2.00),
        ("z1_door_east",      3.65, -11.00, 1.00, 4.70, 0.20, 2.00),

        # Zone 2 Pinch Divider at y = -7.00 (0.85m gap at x=-2.00)
        ("z2_pinch_west",    -4.2125, -7.00, 1.00, 3.575, 0.20, 2.00),
        ("z2_pinch_east",     2.2125, -7.00, 1.00, 7.575, 0.20, 2.00),

        # Zone 3 Clutter Posts (9 posts 0.20x0.20x1.20)
        ("z3_post_01", -4.10, -6.10, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_02", -2.30, -5.40, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_03", -0.40, -6.30, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_04",  1.50, -5.60, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_05",  3.60, -6.20, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_06", -3.40, -3.90, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_07", -1.20, -3.20, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_08",  0.90, -4.10, 0.60, 0.20, 0.20, 1.20),
        ("z3_post_09",  3.10, -3.40, 0.60, 0.20, 0.20, 1.20),

        # Zone 4 Blind Bend Walls
        ("z4_wall_a", -1.65,  0.50, 1.00, 0.20, 5.00, 2.00),
        ("z4_wall_b", -0.35, -0.60, 1.00, 0.20, 2.80, 2.00),
        ("z4_wall_c",  1.60,  0.85, 1.00, 4.30, 0.20, 2.00),
        ("z4_wall_d",  1.60,  2.05, 1.00, 4.30, 0.20, 2.00),

        # Zone 5 Entry Doorway at y = 3.00 (2.20m gap at x=1.50)
        ("z5_door_west", -2.80, 3.00, 1.00, 6.40, 0.20, 2.00),
        ("z5_door_east",  4.30, 3.00, 1.00, 3.40, 0.20, 2.00),

        # Zone 6 Cabinets
        ("z6_cab_west", -4.20, 14.60, 0.50, 1.20, 0.80, 1.00),
        ("z6_cab_east",  4.20, 14.60, 0.50, 1.20, 0.80, 1.00),
    ]

    # Rasterize boxes onto 2D occupancy grid
    for _, bx, by, _, bdx, bdy, _ in boxes:
        min_x, max_x = bx - bdx / 2.0, bx + bdx / 2.0
        min_y, max_y = by - bdy / 2.0, by + bdy / 2.0

        ix_start = max(0, int(round((min_x - origin_x) / res)))
        ix_end   = min(width, int(round((max_x - origin_x) / res)))
        iy_start = max(0, int(round((min_y - origin_y) / res)))
        iy_end   = min(height, int(round((max_y - origin_y) / res)))

        for iy in range(iy_start, iy_end):
            for ix in range(ix_start, ix_end):
                grid[iy][ix] = 0  # 0 represents occupied cell in PGM

    # Ensure perimeter border cells are explicitly occupied walls
    for ix in range(width):
        grid[0][ix] = 0
        grid[1][ix] = 0
        grid[height - 1][ix] = 0
        grid[height - 2][ix] = 0
    for iy in range(height):
        grid[iy][0] = 0
        grid[iy][1] = 0
        grid[iy][width - 1] = 0
        grid[iy][width - 2] = 0

    # Write PGM and YAML to all registered maps directories
    yaml_content = (
        f"image: {map_name}.pgm\n"
        f"mode: trinary\n"
        f"resolution: {res:.4f}\n"
        f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0000]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )

    for mdir in MAPS_DIRS:
        pgm_path = os.path.join(mdir, f"{map_name}.pgm")
        yaml_path = os.path.join(mdir, f"{map_name}.yaml")

        with open(pgm_path, "w") as f:
            f.write("P2\n")
            f.write(f"# Map generated for {map_name}\n")
            f.write(f"{width} {height}\n")
            f.write("255\n")
            for row in reversed(grid):
                f.write(" ".join(str(val) for val in row) + "\n")

        with open(yaml_path, "w") as f:
            f.write(yaml_content)

        print(f"Generated 2D map: {yaml_path} ({width}x{height} px, res={res}m/px)")

    # Build SDF string
    models_sdf = "\n".join(create_wall_box_sdf(b[0], b[1], b[2], b[3], b[4], b[5], b[6]) for b in boxes)

    world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="causal_benchmark_2">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
{models_sdf}
  </world>
</sdf>
"""

    for wdir in WORLDS_DIRS:
        wpath = os.path.join(wdir, f"{map_name}.world")
        with open(wpath, "w") as f:
            f.write(world_sdf)
        print(f"Generated world file: {wpath} ✓")


if __name__ == "__main__":
    generate_world_2()
