#!/usr/bin/env python3
"""
Generator for causal_benchmark_3_dynamic World, SDF, PGM, and YAML map files.
Includes non-static red cylinder model w3_dynamic_obstacle_1 in Gazebo world
for testing dynamic obstacle speed adaptation.
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
    """Generate SDF string for a 3D table model."""
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


def create_dynamic_obstacle_sdf(name: str = "w3_dynamic_obstacle_1", x: float = 3.0, y: float = 5.15, z: float = 0.5) -> str:
    """Generate SDF string for a dynamic non-static cylinder obstacle in Gazebo."""
    return f"""
    <model name='{name}'>
      <static>0</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name='link'>
        <kinematic>1</kinematic>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>1.0</iyy><iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>0.3000</radius>
              <length>1.0000</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>0.3000</radius>
              <length>1.0000</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def generate_world_3_dynamic():
    map_name = "causal_benchmark_3_dynamic"
    
    res = 0.05
    origin_x = -5.0
    origin_y = -11.5
    width = int(round(10.0 / res))   # 200
    height = int(round(23.0 / res))  # 460

    grid = [[255 for _ in range(width)] for _ in range(height)]

    boxes = [
        ("outer_wall_west",  -4.90,  0.00, 1.00, 0.20, 22.80, 2.00),
        ("outer_wall_east",   4.90,  0.00, 1.00, 0.20, 22.80, 2.00),
        ("outer_wall_south",  0.00, -11.40, 1.00, 10.00, 0.20, 2.00),
        ("outer_wall_north",  0.00,  11.40, 1.00, 10.00, 0.20, 2.00),

        ("funnel_side_west",  -3.20, -7.00, 1.00, 3.40, 0.20, 2.00),
        ("funnel_side_east",   3.20, -7.00, 1.00, 3.40, 0.20, 2.00),

        ("div2_west",        -2.70, -3.50, 1.00, 4.20, 0.20, 2.00),
        ("div2_east",         2.70, -3.50, 1.00, 4.20, 0.20, 2.00),

        ("slalom_box_1",     -1.50, -2.00, 0.40, 0.80, 0.80, 0.80),
        ("slalom_box_2",      1.50, -1.00, 0.40, 0.80, 0.80, 0.80),
        ("slalom_box_4",      1.20,  1.20, 0.40, 0.80, 0.80, 0.80),

        ("div3_west",        -2.70,  2.50, 1.00, 4.20, 0.20, 2.00),
        ("div3_east",         2.70,  2.50, 1.00, 4.20, 0.20, 2.00),

        ("maze_wall_1",       0.00,  4.50, 1.00, 0.20, 1.10, 2.00),
        ("maze_wall_2",       0.64,  5.15, 1.00, 3.40, 0.20, 2.00),

        ("div4_west",        -3.30,  6.30, 1.00, 3.00, 0.20, 2.00),
        ("div4_east",         2.05,  6.30, 1.00, 5.50, 0.20, 2.00),
    ]

    angled_boxes = [
        ("funnel_taper_west", -1.025, -5.80, 1.00, 2.58, 0.20, 2.00, 1.200),
        ("funnel_taper_east",  1.025, -5.80, 1.00, 2.58, 0.20, 2.00, -1.200),
    ]

    real_tables = [
        ("w3_table_pick_a",  -3.80, -9.50, 0.375, 0.80, 0.50, 0.75),
        ("w3_table_pick_b",   3.80, -9.50, 0.375, 0.80, 0.50, 0.75),
        ("w3_table_place_a", -3.80,  9.50, 0.375, 0.80, 0.50, 0.75),
        ("w3_table_place_b",  3.80,  9.50, 0.375, 0.80, 0.50, 0.75),
    ]

    unmapped_boxes = [
        ("unmapped_slalom_box_3",     -0.20,  0.20, 0.40, 0.80, 0.80, 0.80),
    ]

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

    sdf_models = [create_wall_box_sdf(*b) for b in boxes]
    sdf_models.extend([create_angled_wall_box_sdf(*b) for b in angled_boxes])
    sdf_models.extend([create_real_table_sdf(*t) for t in real_tables])
    sdf_models.extend([create_wall_box_sdf(*b) for b in unmapped_boxes])
    sdf_models.append(create_dynamic_obstacle_sdf("w3_dynamic_obstacle_1", x=3.0, y=3.50, z=0.5))

    world_sdf = f"""<sdf version='1.6'>
  <world name='causal_benchmark_3_dynamic'>
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
        _write_map_pair(mdir, map_name, f"{map_name}.pgm")
        sub_dir = os.path.join(mdir, map_name)
        _write_map_pair(sub_dir, "map", "map.pgm")
        _write_map_pair(sub_dir, map_name, f"{map_name}.pgm")

    print(f"Successfully generated {map_name} World, PGM, and YAML map files across all formats.")


if __name__ == "__main__":
    generate_world_3_dynamic()
