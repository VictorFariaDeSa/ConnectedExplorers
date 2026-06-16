'''
********************************************************************************
* Multi navigation Launch file
    - This launch file is used to generate multiple robots and control them
********************************************************************************
'''

import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command
import json

'''
********************************************************************************
* User defined params
********************************************************************************
'''
# SCORE CALCULATIONS
SIGHT_SCORE_OFFSET = 0.5
SIGHT_SCORE_SCALE = -6.0

DISTANCE_SCORE_OFFSET = 6.0
DISTANCE_SCORE_SCALE = 1.0

'''
********************************************************************************
* Robots list
********************************************************************************
'''
# Scenario 03:
robots = [
    {"name":"robot1","x":"-6","y":"7.5","function":"task"},
    {"name":"robot2","x":"-6","y":"8.5","function":"task"},
    {"name":"robot3","x":"-7","y":"7.5","function":"conn"},
    {"name":"robot4","x":"-7","y":"8.5","function":"conn"},
]
world_name = 'proj_world.world'
map_name = 'my_map.yaml'

lifecycle_managed_nodes = ["map_server"]

def get_all_robot_names(robots):
    return [robot["name"] for robot in robots ]

def get_robots_functions(robots):
    robots_dict = {robot["name"]:robot["function"] for robot in robots}
    return json.dumps(robots_dict)

'''
********************************************************************************
* Packages information & Directories
********************************************************************************
'''
description_pkg = 'mobile_robot_description'
bringup_pkg = 'mobile_robot_bringup'

world_path = os.path.join(get_package_share_directory(bringup_pkg), 'worlds', world_name)
rviz_config_file = os.path.join(get_package_share_directory(bringup_pkg), 'config', 'rviz_config.rviz')
urdf_path = os.path.join(get_package_share_directory(description_pkg), 'URDF', 'MineMapper.urdf.xacro')
model_path = os.path.join(get_package_share_directory('mobile_robot_bringup'), 'models')
gazebo_config_path = os.path.join(get_package_share_directory(bringup_pkg), 'config', 'gazebo_bridge.yaml')
map_file = os.path.join(get_package_share_directory(bringup_pkg), 'maps', map_name)

os.environ['GZ_SIM_RESOURCE_PATH'] = os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + model_path

def get_robot_nav_yaml_file(robot_name):
    return os.path.join(get_package_share_directory("mobile_robot_bringup"), "config", f"amcl_config_{robot_name}.yaml")

def get_robot_nav_file(robot_name):
    return os.path.join(get_package_share_directory("mobile_robot_bringup"), "config", f"nav2_planner_config_{robot_name}.yaml")

'''
********************************************************************************
* Launch function
********************************************************************************
'''
def generate_launch_description():
    launch_nodes = []

    # 1. GAZEBO
    launch_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_path} -r'}.items()
    )
    launch_nodes.append(launch_gazebo_node)

    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        output='screen'
    )
    launch_nodes.append(gazebo_bridge_node)

    # 2. RVIZ
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{"use_sim_time": True}]
    )
    launch_nodes.append(launch_rviz_node)

    # 3. MAP SERVER
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"topic_name": "map"},
            {"frame_id": "map"},
            {"yaml_filename": map_file}
        ]
    )
    launch_nodes.append(map_server)

    # 4. ROBOTS
    for robot in robots:
        name = robot["name"]
        x_pos = robot["x"]
        y_pos = robot["y"]

        group = GroupAction([
            PushRosNamespace(name),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': Command(['xacro ', urdf_path,' robot_ns_arg:=', name]),
                    'use_sim_time': True,
                    'frame_prefix': f'{name}/'
                }]
            ),

            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-name', name,
                    '-topic', 'robot_description',
                    '-x', x_pos,
                    '-y', y_pos,
                    '-z', '0.1'
                ]
            )
        ])
        launch_nodes.append(group)

        # Nav2 Nodes
        amcl_node = Node(
            namespace=name,
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                get_robot_nav_yaml_file(name),
                {
                    'use_sim_time': True,
                    'set_initial_pose': True,
                    'initial_pose.x': float(x_pos),
                    'initial_pose.y': float(y_pos),
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': 0.0
                }
            ]
        )
        launch_nodes.append(amcl_node)

        planner_node = Node(
            namespace=name,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[get_robot_nav_file(name), {'use_sim_time': True}]
        )
        launch_nodes.append(planner_node)

        controller_node = Node(
            namespace=name,
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[get_robot_nav_file(name), {'use_sim_time': True}],
            remappings=[('cmd_vel', 'ideal_cmd_vel')]
        )
        launch_nodes.append(controller_node)

        behavior_node = Node(
            namespace=name,
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[get_robot_nav_file(name), {'use_sim_time': True}],
            remappings=[('cmd_vel', 'ideal_cmd_vel')]
        )
        launch_nodes.append(behavior_node)

        bt_navigator_node = Node(
            namespace=name,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[get_robot_nav_file(name), {'use_sim_time': True}]
        )
        launch_nodes.append(bt_navigator_node)

        # Append to lifecycle manager
        lifecycle_managed_nodes.extend([
            f"{name}/amcl",
            f"{name}/planner_server",
            f"{name}/controller_server",
            f"{name}/behavior_server",
            f"{name}/bt_navigator"
        ])

        gatekeeper_node = Node(
            namespace=name,
            package='line_viewer', # Or whichever package you put the python scripts in
            executable='sparse_scan_transmitter',
            name='sparse_transmitter',
            output='screen'
        )
        launch_nodes.append(gatekeeper_node)


    # 5. LIFECYCLE MANAGER (Launched immediately, no timer)
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"bond_timeout": 0.0},
            {"node_names": lifecycle_managed_nodes},
        ]
    )
    launch_nodes.append(lifecycle_manager_node)

    # 6. CUSTOM LINE VIEWER NODES
    marker_node1 = Node(
        package="line_viewer",
        executable="RobotsPositionNode",
        name="RobotsPositionNode",
        parameters=[
            {"use_sim_time": True},
            {"robots_list": get_all_robot_names(robots)},
            {"reference_frame": "map"}
        ]
    )
    launch_nodes.append(marker_node1)

    marker_node2 = Node(
        package="line_viewer",
        executable="RobotsMathNode",
        name="RobotsMathNode",
        parameters=[
            {"use_sim_time": True},
            {"robots_list": get_all_robot_names(robots)},
            {"sight_score_offset": SIGHT_SCORE_OFFSET},
            {"sight_score_scale": SIGHT_SCORE_SCALE},
            {"distance_score_offset": DISTANCE_SCORE_OFFSET},
            {"distance_score_scale": DISTANCE_SCORE_SCALE},
            {"laplacian_topic_name": "laplacian_matrix"},
        ]
    )
    launch_nodes.append(marker_node2)

    marker_node3 = Node(
        package="line_viewer",
        executable="SightMarkerNode",
        name="SightMarkerNode",
        parameters=[
            {"use_sim_time": True},
            {"robots_list": get_all_robot_names(robots)},
            {"reference_frame": "map"},
            {"publisher_node_name": "visualization_marker"}
        ]
    )
    launch_nodes.append(marker_node3)

    marker_node4 = Node(
        package="line_viewer",
        executable="RobotsControllerNode",
        name="RobotsControllerNode",
        parameters=[
            {"use_sim_time": True},
            {"robots_list": get_all_robot_names(robots)},
            {'robots_function_map': get_robots_functions(robots)}
        ]
    )
    launch_nodes.append(marker_node4)

    supervisor_node = Node(
            package='line_viewer',
            executable='supervisor_mapper',
            name='supervisor_mapper',
            output='screen'
        )
    launch_nodes.append(supervisor_node)

    # Add this near your supervisor_node in the launch file
    launch_nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot1/map'] # Adjust or just link map to map
    ))


    return LaunchDescription(launch_nodes)
