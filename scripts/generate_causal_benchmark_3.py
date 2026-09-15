#!/usr/bin/env python3
"""
Generator for causal_benchmark_3 World (World 3), SDF, PGM, and YAML map files.
Includes 4 real placement tables (0.80 x 0.50 x 0.75 m) in Z1 and Z6:
  w3_table_pick_a   pose (-3.00, -14.00, 0.375)
  w3_table_pick_b   pose ( 3.00, -14.00, 0.375)
  w3_table_place_a  pose (-3.00,  14.00, 0.375)
  w3_table_place_b  pose ( 3.00,  14.00, 0.375)
"""

import os
import sys
import math

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
    "/opt/ros/humble/share/pal_gazebo_worlds/worlds",
    "/opt/ros/humble/share/gazebo_simulation/worlds",
]
MAPS_DIRS = [
    os.path.join(GAZEBO_SIM_DIR, "maps"),
    os.path.join(WORKSPACE_ROOT, "install", "gazebo_simulation", "share", "gazebo_simulation", "maps"),
    os.path.join(WORKSPACE_ROOT, "src", "install", "gazebo_simulation", "share", "gazebo_simulation", "maps"),
    "/opt/ros/humble/share/pal_maps/maps",
    "/opt/ros/humble/share/gazebo_simulation/maps",
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


def create_angled_wall_box_sdf(name: str, x: float, y: float, z: float, dx: float, dy: float, dz: float = 2.0, yaw: float = 0.0) -> str:
    """Generate SDF string for a static wall/obstacle box model with yaw orientation."""
    return f"""
    <model name='{name}'>
      <static>1</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
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


def create_real_table_sdf(name: str, x: float, y: float, z: float = 0.375, dx: float = 0.80, dy: float = 0.50, dz: float = 0.75) -> str:
    """Generate SDF string for a beautiful 3D table model with wooden top and metal legs."""
    top_z = dz - 0.02
    leg_h = dz - 0.04
    leg_z = leg_h / 2.0
    lx = (dx / 2.0) - 0.05
    ly = (dy / 2.0) - 0.05
    
    return f"""
    <model name="{name}">
      <static>1</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <pose>0 0 {dz/2.0:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{dx:.4f} {dy:.4f} {dz:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="top_visual">
          <pose>0 0 {top_z:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{dx:.4f} {dy:.4f} 0.0400</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/WoodFloor</name>
            </script>
          </material>
        </visual>
        <visual name="leg1_visual">
          <pose>{lx:.4f} {ly:.4f} {leg_z:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>{leg_h:.4f}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/FlatBlack</name>
            </script>
          </material>
        </visual>
        <visual name="leg2_visual">
          <pose>{-lx:.4f} {ly:.4f} {leg_z:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>{leg_h:.4f}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/FlatBlack</name>
            </script>
          </material>
        </visual>
        <visual name="leg3_visual">
          <pose>{lx:.4f} {-ly:.4f} {leg_z:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>{leg_h:.4f}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/FlatBlack</name>
            </script>
          </material>
        </visual>
        <visual name="leg4_visual">
          <pose>{-lx:.4f} {-ly:.4f} {leg_z:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.025</radius>
              <length>{leg_h:.4f}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/FlatBlack</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world_3():
    map_name = "causal_benchmark_3"
    
    res = 0.05
    origin_x = -5.0
    origin_y = -11.5
    width = int(round(10.0 / res))   # 200
    height = int(round(23.0 / res))  # 460

    grid = [[255 for _ in range(width)] for _ in range(height)]

    boxes = [
        # Outer perimeter walls (10m wide x 23m long map)
        ("outer_wall_west",  -4.90,  0.00, 1.00, 0.20, 22.80, 2.00),
        ("outer_wall_east",   4.90,  0.00, 1.00, 0.20, 22.80, 2.00),
        ("outer_wall_south",  0.00, -11.40, 1.00, 10.00, 0.20, 2.00),
        ("outer_wall_north",  0.00,  11.40, 1.00, 10.00, 0.20, 2.00),

        # ------------------------------------------------------------------
        # Zone 1 Funnel Side Closure Walls (y = -7.00)
        # Closes off the sides of the 3.0m mouth, forcing robot into the funnel
        # ------------------------------------------------------------------
        ("funnel_side_west",  -3.20, -7.00, 1.00, 3.40, 0.20, 2.00),
        ("funnel_side_east",   3.20, -7.00, 1.00, 3.40, 0.20, 2.00),

        # ------------------------------------------------------------------
        # Zone 2 -> Zone 3 Transition (y = -3.50)
        # Doorway A: 1.10m bottleneck doorway (x in [-0.55, 0.55])
        # Carry footprint (1.48m) CANNOT pass! Forces arm tucking.
        # ------------------------------------------------------------------
        ("div2_west",        -2.70, -3.50, 1.00, 4.20, 0.20, 2.00),
        ("div2_east",         2.70, -3.50, 1.00, 4.20, 0.20, 2.00),

        # ------------------------------------------------------------------
        # Zone 3: Mapped Slalom Field Obstacles (y in [-3.5, 2.5])
        # High obstacle density r_dens > 0.05, tight clearance r_clear < 0.6m
        # ------------------------------------------------------------------
        ("slalom_box_1",     -1.50, -2.00, 0.40, 0.80, 0.80, 0.80),
        ("slalom_box_2",      1.50, -1.00, 0.40, 0.80, 0.80, 0.80),
        ("slalom_box_4",      1.20,  1.20, 0.40, 0.80, 0.80, 0.80),

        # ------------------------------------------------------------------
        # Zone 3 -> Zone 4 Transition (y = 2.50)
        # Doorway B: 1.10m bottleneck doorway (x in [-0.55, 0.55])
        # ------------------------------------------------------------------
        ("div3_west",        -2.70,  2.50, 1.00, 4.20, 0.20, 2.00),
        ("div3_east",         2.70,  2.50, 1.00, 4.20, 0.20, 2.00),

        # ------------------------------------------------------------------
        # Zone 4: 90-Degree Sharp Turn Maze Corridor (Mapped in .pgm map)
        # maze_wall_1 spans y in [3.80, 4.90], meeting maze_wall_2 (y in [4.90, 5.10])
        # without extending north past maze_wall_2.
        # Entrance gap above Doorway B (y = 2.60 to 3.80) = 1.20m clear passage.
        # Horizontal maze channel width (y = 5.10 to 6.20) = 1.10m clear corridor.
        # ------------------------------------------------------------------
        ("maze_wall_1",       0.00,  4.50, 1.00, 0.20, 1.10, 2.00),
        ("maze_wall_2",       0.64,  5.15, 1.00, 3.40, 0.20, 2.00),

        # ------------------------------------------------------------------
        # Zone 4 -> Zone 5 Transition (y = 6.30)
        # Doorway C: 1.10m bottleneck doorway (x in [-1.80, -0.70])
        # ------------------------------------------------------------------
        ("div4_west",        -3.30,  6.30, 1.00, 3.00, 0.20, 2.00),
        ("div4_east",         2.05,  6.30, 1.00, 5.50, 0.20, 2.00),
    ]

    # Angled wall boxes for the south room funnel (monotone taper from mouth 3.0m at y=-7.0 to throat 1.10m at y=-4.6)
    angled_boxes = [
        ("funnel_taper_west", -1.025, -5.80, 1.00, 2.58, 0.20, 2.00, 1.200),
        ("funnel_taper_east",  1.025, -5.80, 1.00, 2.58, 0.20, 2.00, -1.200),
    ]

    # Real 3D placement tables placed in Zone 1 (Pick) and Zone 5 (Place)
    real_tables = [
        ("w3_table_pick_a",  -3.80, -9.50, 0.375, 0.80, 0.50, 0.75),
        ("w3_table_pick_b",   3.80, -9.50, 0.375, 0.80, 0.50, 0.75),
        ("w3_table_place_a", -3.80,  9.50, 0.375, 0.80, 0.50, 0.75),
        ("w3_table_place_b",  3.80,  9.50, 0.375, 0.80, 0.50, 0.75),
    ]

    # Unmapped dynamic obstacle boxes (present in Gazebo .world SDF ONLY, NOT in .pgm/.yaml static map).
    # unmapped_slalom_box_3 at (-0.20, 0.20) sits directly on the straight-line path between div2 & div3.
    unmapped_boxes = [
        ("unmapped_slalom_box_3",     -0.20,  0.20, 0.40, 0.80, 0.80, 0.80),
    ]

    # Combine mapped boxes and real tables into raster grid (unmapped_boxes ARE EXCLUDED)
    all_raster_boxes = list(boxes) + [(t[0], t[1], t[2], t[3], t[4], t[5], t[6]) for t in real_tables]

    for name, bx, by, bz, bdx, bdy, bdz in all_raster_boxes:
        min_x = bx - bdx / 2.0
        max_x = bx + bdx / 2.0
        min_y = by - bdy / 2.0
        max_y = by + bdy / 2.0

        col_start = max(0, int(math.floor((min_x - origin_x) / res)))
        col_end = min(width - 1, int(math.ceil((max_x - origin_x) / res)))
        row_start = max(0, int(math.floor((min_y - origin_y) / res)))
        row_end = min(height - 1, int(math.ceil((max_y - origin_y) / res)))

        for r in range(row_start, row_end + 1):
            for c in range(col_start, col_end + 1):
                grid[r][c] = 0

    # Rasterize angled funnel taper walls cell-by-cell into the .pgm grid
    for name, bx, by, bz, bdx, bdy, bdz, yaw in angled_boxes:
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        half_dx = bdx / 2.0
        half_dy = bdy / 2.0
        corners_x = [-half_dx, half_dx, -half_dx, half_dx]
        corners_y = [-half_dy, -half_dy, half_dy, half_dy]
        rot_x = [bx + cx * math.cos(yaw) - cy * math.sin(yaw) for cx, cy in zip(corners_x, corners_y)]
        rot_y = [by + cx * math.sin(yaw) + cy * math.cos(yaw) for cx, cy in zip(corners_x, corners_y)]
        min_x, max_x = min(rot_x), max(rot_x)
        min_y, max_y = min(rot_y), max(rot_y)

        col_start = max(0, int(math.floor((min_x - origin_x) / res)))
        col_end = min(width - 1, int(math.ceil((max_x - origin_x) / res)))
        row_start = max(0, int(math.floor((min_y - origin_y) / res)))
        row_end = min(height - 1, int(math.ceil((max_y - origin_y) / res)))

        for r in range(row_start, row_end + 1):
            cy_m = origin_y + (r + 0.5) * res
            dy_c = cy_m - by
            for c in range(col_start, col_end + 1):
                cx_m = origin_x + (c + 0.5) * res
                dx_c = cx_m - bx
                lx = dx_c * cos_y - dy_c * sin_y
                ly = dx_c * sin_y + dy_c * cos_y
                if abs(lx) <= half_dx + 0.02 and abs(ly) <= half_dy + 0.02:
                    grid[r][c] = 0

    # Build SDF String (includes mapped walls, angled funnel walls, tables, AND unmapped runtime obstacles)
    sdf_models = [create_wall_box_sdf(*b) for b in boxes]
    sdf_models.extend([create_angled_wall_box_sdf(*b) for b in angled_boxes])
    sdf_models.extend([create_real_table_sdf(*t) for t in real_tables])
    sdf_models.extend([create_wall_box_sdf(*b) for b in unmapped_boxes])

    world_sdf = f"""<sdf version='1.6'>
  <world name='causal_benchmark_3'>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin name='gazebo_occupancy_grid' filename='libgazebo_occupancy_grid.so'/>
    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'/>
    <plugin name="collision_monitor" filename="libcollision_monitor.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <robot_name>tiago</robot_name>
      <ignore_models>ground_plane</ignore_models>
      <force_threshold>1.0</force_threshold>
      <publish_rate>50</publish_rate>
    </plugin>

    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    {"".join(sdf_models)}
  </world>
</sdf>
"""

    for wdir in WORLDS_DIRS:
        world_file = os.path.join(wdir, f"{map_name}.world")
        with open(world_file, "w") as f:
            f.write(world_sdf)
        print(f"Wrote SDF world to: {world_file}")

    pgm_bytes = bytearray()
    for r in reversed(range(height)):
        pgm_bytes.extend(grid[r])

    header = f"P5\n{width} {height}\n255\n".encode("ascii")

    def _write_map_pair(directory: str, name_prefix: str, image_filename: str):
        try:
            os.makedirs(directory, exist_ok=True)
            pgm_f = os.path.join(directory, f"{name_prefix}.pgm")
            yaml_f = os.path.join(directory, f"{name_prefix}.yaml")
            with open(pgm_f, "wb") as f:
                f.write(header)
                f.write(pgm_bytes)
            yaml_body = f"""image: {image_filename}
mode: trinary
resolution: {res}
origin: [{origin_x:.2f}, {origin_y:.2f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
            with open(yaml_f, "w") as f:
                f.write(yaml_body)
        except PermissionError:
            pass

    for mdir in MAPS_DIRS:
        # Write flat named format: causal_benchmark_3.yaml -> causal_benchmark_3.pgm
        _write_map_pair(mdir, map_name, f"{map_name}.pgm")
        # Write flat fallback: map.yaml -> map.pgm
        _write_map_pair(mdir, "map", "map.pgm")
        # Write subdirectory format: causal_benchmark_3/map.yaml -> map.pgm (used by PAL nav)
        sub_dir = os.path.join(mdir, map_name)
        _write_map_pair(sub_dir, "map", "map.pgm")
        _write_map_pair(sub_dir, map_name, f"{map_name}.pgm")

    print(f"Successfully generated {map_name} World, PGM, and YAML map files across all formats.")


if __name__ == "__main__":
    generate_world_3()
