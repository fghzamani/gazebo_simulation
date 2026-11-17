# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import tempfile
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_pal.include_utils import (
    include_scoped_launch_py_description,
    include_launch_py_description,
)
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from dataclasses import dataclass
from launch_pal.robot_arguments import CommonArgs
from launch_ros.actions import Node
from tiago_description.launch_arguments import TiagoArgs
from launch_pal.actions import CheckPublicSim


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    has_screen: DeclareLaunchArgument = TiagoArgs.has_screen
    arm_type: DeclareLaunchArgument = TiagoArgs.arm_type
    arm_motor_model: DeclareLaunchArgument = TiagoArgs.arm_motor_model
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    ft_sensor: DeclareLaunchArgument = TiagoArgs.ft_sensor
    wrist_model: DeclareLaunchArgument = TiagoArgs.wrist_model
    camera_model: DeclareLaunchArgument = TiagoArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoArgs.laser_model

    navigation: DeclareLaunchArgument = CommonArgs.navigation
    gzclient: DeclareLaunchArgument = CommonArgs.gzclient
    # RViz off for headless runs: nav2_bringup's rviz_launch shuts the WHOLE
    # launch down when RViz exits, so a crashed RViz would kill a campaign.
    rviz: DeclareLaunchArgument = DeclareLaunchArgument(
        'rviz', default_value='True', description='Start RViz with navigation')
    nav_params: DeclareLaunchArgument = DeclareLaunchArgument(
        'nav_params', default_value='',
        description='Nav2 params file (public sim). Default: pmb2_2dnav/config/'
                    'nav_campaign2.yaml for worlds pal_office_campaign2 and '
                    'causal_benchmark*, else nav_public_sim.yaml')
    advanced_navigation: DeclareLaunchArgument = CommonArgs.advanced_navigation
    slam: DeclareLaunchArgument = CommonArgs.slam
    docking: DeclareLaunchArgument = CommonArgs.docking
    moveit: DeclareLaunchArgument = CommonArgs.moveit
    world_name: DeclareLaunchArgument = CommonArgs.world_name
    namespace: DeclareLaunchArgument = CommonArgs.namespace
    tuck_arm: DeclareLaunchArgument = CommonArgs.tuck_arm
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim
    
    #custom launch arguments for robot spawn position added by fgh
    x_pose: DeclareLaunchArgument = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='X position for robot spawn')
    y_pose: DeclareLaunchArgument = DeclareLaunchArgument(
        'y_pose', default_value='-8.5', description='Y position for robot spawn')
    yaw_pose: DeclareLaunchArgument = DeclareLaunchArgument(
        'yaw_pose', default_value='1.5708', description='Yaw orientation for robot spawn')
    map_path: DeclareLaunchArgument = DeclareLaunchArgument(
        'map_path', default_value='', description='Path to custom 2D map.yaml file')
    map: DeclareLaunchArgument = DeclareLaunchArgument(
        'map', default_value='', description='Path to custom 2D map.yaml file (alias for map_path)')


def private_navigation(context, *args, **kwargs):
    actions = []
    base_type = read_launch_argument('base_type', context)
    camera_model = read_launch_argument('camera_model', context)
    docking = read_launch_argument('docking', context)
    advanced_navigation = read_launch_argument('advanced_navigation', context)
    use_sim_time = read_launch_argument('use_sim_time', context)
    rviz_cfg_pkg = base_type + '_2dnav'
    if advanced_navigation == 'True':
        rviz_cfg_pkg = base_type + '_advanced_2dnav'

    robot_info = {
        "robot_info_publisher": {
            "ros__parameters": {
                "robot_type": "tiago",
                "base_type": base_type,
                "laser_model": "sick-571",
                "camera_model": camera_model,
                "advanced_navigation": (advanced_navigation == 'True'),
                "has_dock": (docking == 'True'),
                "use_sim_time": (use_sim_time == 'True'),
            }
        }
    }

    temp_yaml = tempfile.mkdtemp()
    temp_robot_info = os.path.join(temp_yaml, '99_robot_info.yaml')
    with open(temp_robot_info, 'w') as temp_robot_info_file:
        yaml.safe_dump(robot_info, temp_robot_info_file)

    # Robot Info Publisher
    robot_info_env = SetEnvironmentVariable(
        name='ROBOT_INFO_PATH',
        value=temp_yaml,
    )
    actions.append(robot_info_env)

    robot_info_publisher = Node(
        package='robot_info_publisher',
        executable='robot_info_publisher',
        name='robot_info_publisher',
        output='screen',
    )
    actions.append(robot_info_publisher)

    # Laser Sensors
    laser_bringup_launch = include_launch_py_description(
        pkg_name=base_type + '_laser_sensors',
        paths=['launch', 'laser_sim.launch.py'],
    )
    actions.append(laser_bringup_launch)

    # Navigation
    nav_bringup_launch = include_launch_py_description(
        pkg_name=base_type + '_2dnav',
        paths=['launch', 'navigation.launch.py'],
    )
    actions.append(nav_bringup_launch)

    # Localization
    loc_bringup_launch = include_launch_py_description(
        pkg_name=base_type + '_2dnav',
        paths=['launch', 'localization.launch.py'],
        condition=UnlessCondition(LaunchConfiguration('slam'))
    )
    actions.append(loc_bringup_launch)

    # SLAM
    slam_bringup_launch = include_launch_py_description(
        pkg_name=base_type + '_2dnav',
        paths=['launch', 'slam.launch.py'],
        condition=IfCondition(LaunchConfiguration('slam'))
    )
    actions.append(slam_bringup_launch)

    # Docking
    docking_bringup_launch = include_launch_py_description(
        pkg_name=base_type + '_docking',
        paths=['launch', 'docking_sim.launch.py'],
        condition=IfCondition(LaunchConfiguration('docking'))
    )
    actions.append(docking_bringup_launch)

    # Stores Server
    db_bringup_launch = Node(
        package='pal_stores_server',
        executable='pal_stores_server',
        arguments=[os.path.join(
            os.environ['HOME'], '.pal', 'stores.db'
        )],
        output='screen',
        condition=IfCondition(LaunchConfiguration('advanced_navigation'))
    )
    actions.append(db_bringup_launch)

    # Advanced Navigation
    advanced_nav_bringup_launch = include_launch_py_description(
        pkg_name=base_type + '_advanced_2dnav',
        paths=['launch', 'advanced_navigation.launch.py'],
        condition=IfCondition(LaunchConfiguration('advanced_navigation'))
    )
    actions.append(advanced_nav_bringup_launch)

    # RViz
    rviz_bringup_launch = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(rviz_cfg_pkg),
            'config',
            'rviz',
            'navigation.rviz',
        )],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    actions.append(rviz_bringup_launch)
    return actions


def public_navigation(context, *args, **kwargs):
    actions = []
    base_type = read_launch_argument('base_type', context)
    base_2dnav = get_package_share_directory(base_type + '_2dnav')
    pal_maps = get_package_share_directory('pal_maps')
    world_name_raw = read_launch_argument('world_name', context)
    world_basename = os.path.splitext(os.path.basename(world_name_raw))[0]
    # Nav2 params: explicit nav_params:=, else the campaign-2 file for the
    # campaign-2 world (the configuration campaign A2 ran with; the collector
    # captures its baseline from it) AND for the causal_benchmark* worlds,
    # else PAL's nav_public_sim.yaml.
    # causal_benchmark* added 2026-11-17: the tuner model is trained on
    # campaign-2 data, so it must be tested on the same Nav2 config (its
    # risk features read the 5x5 m local costmap; nav_public_sim.yaml has
    # 3x3 m, cost_scaling 10, other MPPI weights and PAL's recovery tree).
    # nav_campaign2.yaml's AMCL initial pose (0, -8.5, pi/2) equals this
    # launch's default spawn. The ICRA Campaign B benchmark ran these worlds on
    # nav_public_sim.yaml; reproduce it with nav_params:=<...>/nav_public_sim.yaml.
    given_params = (read_launch_argument('nav_params', context) or '').strip()
    if given_params:
        param_file = given_params
    elif (world_basename == 'pal_office_campaign2'
          or world_basename.startswith('causal_benchmark')):
        param_file = os.path.join(base_2dnav, 'config', 'nav_campaign2.yaml')
    else:
        param_file = os.path.join(base_2dnav, 'config', 'nav_public_sim.yaml')
    if not os.path.exists(param_file):
        raise FileNotFoundError(
            f"Nav2 params file not found: {param_file} (rebuild pmb2_2dnav?)")
    print(f"[gazebo_with_navigation] Nav2 params: '{param_file}'")

    custom_map_path = read_launch_argument('map_path', context)
    custom_map = read_launch_argument('map', context)
    given_map = (custom_map_path or custom_map or "").strip()

    gazebo_sim_maps = os.path.join(get_package_share_directory('gazebo_simulation'), 'maps')
    possible_map_1 = os.path.join(gazebo_sim_maps, f"{world_basename}.yaml")
    possible_map_2 = os.path.join(gazebo_sim_maps, world_basename, 'map.yaml')
    pal_map = os.path.join(pal_maps, 'maps', world_basename, 'map.yaml')

    if given_map and os.path.exists(given_map):
        map_path = given_map
    elif os.path.exists(possible_map_1):
        map_path = possible_map_1
    elif os.path.exists(possible_map_2):
        map_path = possible_map_2
    else:
        map_path = pal_map

    print(f"[gazebo_with_navigation] Resolved 2D map path: '{map_path}' (exists: {os.path.exists(map_path)})")

    # Navigation
    nav2_bringup_launch = include_scoped_launch_py_description(
        pkg_name='nav2_bringup',
        paths=['launch', 'navigation_launch.py'],
        launch_arguments={
            'params_file': param_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'True',
        }
    )
    actions.append(nav2_bringup_launch)

    # Localization
    loc_bringup_launch = include_scoped_launch_py_description(
        pkg_name='nav2_bringup',
        paths=['launch', 'localization_launch.py'],
        launch_arguments={
            'params_file': param_file,
            'map': map_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'True',
        },
        condition=UnlessCondition(LaunchConfiguration('slam')),
    )
    actions.append(loc_bringup_launch)

    # SLAM
    slam_bringup_launch = include_scoped_launch_py_description(
        pkg_name='nav2_bringup',
        paths=['launch', 'slam_launch.py'],
        launch_arguments={
            'params_file': param_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        },
        condition=IfCondition(LaunchConfiguration('slam')),
    )
    actions.append(slam_bringup_launch)

    # RViz
    rviz_bringup_launch = include_scoped_launch_py_description(
        pkg_name='nav2_bringup',
        paths=['launch', 'rviz_launch.py'],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    actions.append(rviz_bringup_launch)
    return actions


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    # Set use_sim_time to True
    set_sim_time = SetLaunchConfiguration("use_sim_time", "True")
    launch_description.add_action(set_sim_time)

    # Shows error if is_public_sim is not set to True when using public simulation
    public_sim_check = CheckPublicSim()
    launch_description.add_action(public_sim_check)

    robot_name = 'tiago'
    packages = ['tiago_description', 'pmb2_description',
                'pal_hey5_description', 'pal_gripper_description',
                'pal_robotiq_description', 'omni_base_description',
                'pal_urdf_utils']

    model_path = get_model_paths(packages)

    gazebo_model_path_env_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path)

    gazebo = include_scoped_launch_py_description(
        pkg_name='pal_gazebo_worlds',
        paths=['launch', 'pal_gazebo.launch.py'],
        env_vars=[gazebo_model_path_env_var],
        launch_arguments={
            "world_name":  launch_args.world_name,
            "gzclient": LaunchConfiguration('gzclient'),
            "model_paths": packages,
            "resource_paths": packages,
            "extra_gazebo_args": "-s libgazebo_ros_state.so",
        })

    launch_description.add_action(gazebo)

    navigation = GroupAction(
        condition=IfCondition(LaunchConfiguration('navigation')),
        actions=[
            # Private Navigation
            OpaqueFunction(
                function=private_navigation,
                condition=UnlessCondition(LaunchConfiguration('is_public_sim'))
            ),
            # Public Navigation
            OpaqueFunction(
                function=public_navigation,
                condition=IfCondition(LaunchConfiguration('is_public_sim'))
            ),
        ]
    )
    launch_description.add_action(navigation)

    move_group = include_scoped_launch_py_description(
        pkg_name='tiago_moveit_config',
        paths=['launch', 'move_group.launch.py'],
        launch_arguments={
            "robot_name": robot_name,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "namespace": launch_args.namespace,
            "base_type": launch_args.base_type,
            "arm_type": launch_args.arm_type,
            "end_effector": launch_args.end_effector,
            "ft_sensor": launch_args.ft_sensor
        },
        condition=IfCondition(LaunchConfiguration('moveit')))

    launch_description.add_action(move_group)

    tiago_bringup = include_scoped_launch_py_description(
        pkg_name='tiago_bringup', paths=['launch', 'tiago_bringup.launch.py'],
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "arm_type": launch_args.arm_type,
            "arm_motor_model": launch_args.arm_motor_model,
            "laser_model": launch_args.laser_model,
            "camera_model": launch_args.camera_model,
            "base_type": launch_args.base_type,
            "wrist_model": launch_args.wrist_model,
            "ft_sensor": launch_args.ft_sensor,
            "end_effector": launch_args.end_effector,
            "has_screen": launch_args.has_screen,
            "is_public_sim": launch_args.is_public_sim,
        }
    )

    launch_description.add_action(tiago_bringup)

    robot_spawn = Node(package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',        robot_name,
            '-topic',         'robot_description',
            '-x',             LaunchConfiguration('x_pose'),
            '-y',             LaunchConfiguration('y_pose'),
            '-Y',             LaunchConfiguration('yaw_pose'),   # capital Y = yaw
        ],
        output='screen'
    )

    launch_description.add_action(robot_spawn)

    tuck_arm = Node(package='tiago_gazebo',
                    executable='tuck_arm.py',
                    emulate_tty=True,
                    output='both',
                    condition=IfCondition(LaunchConfiguration('tuck_arm')))

    launch_description.add_action(tuck_arm)
    
    
    # Risk-state publisher — provides /risk_state (8-dim R_t) for the RCT collector.
    # Gated on navigation because it consumes the local costmap, /amcl_pose and /plan,
    # none of which exist until the nav stack is up.
    launch_description.add_action(DeclareLaunchArgument(
        'risk_state', default_value='True',
        description='Start the rct_collector risk_state_node publishing /risk_state'))

    risk_state_node = Node(
        package='rct_collector',
        executable='risk_state_node',
        name='risk_state_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_topic': '/scan_raw',
            'costmap_topic': '/local_costmap/costmap_raw',
            'pose_topic': '/amcl_pose',
            'odom_topic': '/mobile_base_controller/odom',
            'path_topic': '/plan',
            'compute_rate_hz': 10.0,
        }],
        condition=IfCondition(LaunchConfiguration('risk_state')),
    )
    launch_description.add_action(risk_state_node)

    add_dynamic_obstacle(launch_description)

    return


# Moving obstacles of world_name:=pal_office_campaign2 (pal_gazebo_worlds). The
# world holds them at standby; one dynamic_obstacle_controller per obstacle
# slides it to its blocking pose through /gazebo/set_entity_state. The two
# static stubs are in the world and in gazebo_simulation/maps/
# pal_office_campaign2/map.yaml, which the map lookup above picks automatically
# for that world name. The moving obstacles are NOT in the map.
CAMPAIGN2_DYNAMIC = [
    # obs_dynamic_1: 0.6 m box at the west junction; narrows ~2.05 m to ~1.45 m
    dict(name="obs_dynamic_1", standby=(-4.50, -2.80), blocked=(-4.50, -0.60),
         yaw=0.2723, speed=0.50, trigger=(1.0, 3.2), size=(0.60, 0.60), seed=11,
         reset_srv="/reset_dynamic_obstacle"),
    # obs_dynamic_2: 3.2 m board in the east hall (Wall_77..Wall_150); slides out
    # from behind Wall_28 and closes the 4.46 m hall to a 1.4 m gap. Faster and
    # triggered earlier than #1: it travels 3.15 m and the hall has a long
    # sight line; the distance is to the board centre, ~2.3 m off the gap.
    dict(name="obs_dynamic_2", standby=(11.77, 0.16), blocked=(8.74, -0.68),
         yaw=0.2723, speed=0.80, trigger=(2.5, 5.0), size=(3.20, 0.40), seed=22,
         reset_srv="/reset_dynamic_obstacle_2"),
]

# Moving obstacle of world_name:=causal_benchmark_3_dynamic: the 0.3 m-radius
# cylinder w3_dynamic_obstacle_1 sliding along y = 3.5 between x = 3.0
# (standby) and x = -4.2 (blocked). These are dynamic_obstacle_controller's
# built-in defaults, i.e. the ICRA one-shot benchmark, so dynamic_obstacle_mode:=
# one_shot reproduces the manual `ros2 run` of that study. Not in the map.
BENCHMARK3_DYNAMIC = [
    dict(name="w3_dynamic_obstacle_1", standby=(3.0, 3.5), blocked=(-4.2, 3.5),
         yaw=0.0, speed=0.70, trigger=(1.0, 3.2), trigger_metric="y_line",
         size=(0.60, 0.60), seed=33, reset_srv="/reset_dynamic_obstacle"),
]

# world basename -> obstacles its controllers move. 'auto' = continuous for
# these worlds, off for every other world.
DYNAMIC_OBSTACLES_BY_WORLD = {
    "pal_office_campaign2": CAMPAIGN2_DYNAMIC,
    "causal_benchmark_3_dynamic": BENCHMARK3_DYNAMIC,
}


def add_dynamic_obstacle(launch_description):
    launch_description.add_action(DeclareLaunchArgument(
        'dynamic_obstacle_mode', default_value='auto',
        choices=['auto', 'continuous', 'one_shot', 'oscillating', 'off'],
        description='Moving obstacles of pal_office_campaign2 (obs_dynamic_1, '
                    'obs_dynamic_2) and causal_benchmark_3_dynamic '
                    '(w3_dynamic_obstacle_1): continuous = shuttle standby <-> blocked '
                    'all the time, independent of the robot (random start phase, random '
                    '1-6 s dwell at each end, never resets, backs off instead of driving '
                    'into the robot); one_shot = slide to the blocking pose when the '
                    'robot comes within the trigger band, then hold; oscillating = legacy '
                    'shuttle that resets on every teleport; off = no controller (start '
                    'one by hand if you want); auto = continuous for those two worlds, '
                    'off otherwise'))
    launch_description.add_action(
        OpaqueFunction(function=_dynamic_obstacle_nodes))


def _dynamic_obstacle_nodes(context, *args, **kwargs):
    """One dynamic_obstacle_controller per obstacle of the launched world."""
    mode = read_launch_argument('dynamic_obstacle_mode', context)
    world = os.path.splitext(os.path.basename(
        read_launch_argument('world_name', context)))[0]
    obstacles = DYNAMIC_OBSTACLES_BY_WORLD.get(world, [])
    effective = ('continuous' if obstacles else 'off') if mode == 'auto' else mode
    if effective == 'off' or not obstacles:
        if mode not in ('auto', 'off'):
            print(f"[gazebo_with_navigation] dynamic_obstacle_mode:={mode} ignored: "
                  f"no moving obstacles defined for world '{world}'")
        return []
    print(f"[gazebo_with_navigation] dynamic obstacles ({effective}): "
          f"{', '.join(d['name'] for d in obstacles)}")
    nodes = []
    for dyn in obstacles:
        nodes.append(Node(
            package='online_causal_tuner', executable='dynamic_obstacle_controller',
            name='controller_' + dyn["name"], output='screen',
            remappings=[('/reset_dynamic_obstacle', dyn["reset_srv"])],
            parameters=[{
                'use_sim_time': True,
                'mode': effective,
                'obstacle_name': dyn["name"],
                'robot_name': 'tiago',
                'standby_x': dyn["standby"][0], 'standby_y': dyn["standby"][1],
                'blocked_x': dyn["blocked"][0], 'blocked_y': dyn["blocked"][1],
                'fixed_y': dyn["blocked"][1],
                'yaw': dyn["yaw"],
                'speed_mps': dyn["speed"],
                'trigger_metric': dyn.get("trigger_metric", 'point'),
                'trigger_dist_min': dyn["trigger"][0],
                'trigger_dist_max': dyn["trigger"][1],
                'size_x': dyn["size"][0], 'size_y': dyn["size"][1],
                'seed': dyn["seed"],
                'dwell_min_sec': 1.0, 'dwell_max_sec': 6.0,
                'yield_radius_m': 0.9,
            }]))
    return nodes


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths += pathsep + environ['GAZEBO_MODEL_PATH']

    return model_paths




   