"""
********************************************************************************
* Multi navigation Launch file (Production Architecture)
    - Fully decentralized lifecycle management
    - Parallelized robot initialization
********************************************************************************
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node, PushRosNamespace

"""
********************************************************************************
* User defined params & Robots list
********************************************************************************
"""
DIMENSIONS = 2


SIGHT_SCORE_OFFSET = 0.5
SIGHT_SCORE_SCALE = -6.0

DISTANCE_SCORE_OFFSET = 6.0
DISTANCE_SCORE_SCALE = 1.0

# Scenario 03:
robots = [
    {"name": "robot1", "x": "-6", "y": "7.5", "function": "task"},
    {"name": "robot2", "x": "-6", "y": "8.5", "function": "task"},
    {"name": "robot3", "x": "-7", "y": "7.5", "function": "conn"},
    {"name": "robot4", "x": "-7", "y": "8.5", "function": "conn"},
]
world_name = "proj_world.world"
map_name = "my_map.yaml"

def get_all_robot_names(robots):
    return [robot["name"] for robot in robots]

def get_robots_functions(robots):
    robots_dict = {robot["name"]: robot["function"] for robot in robots}
    return json.dumps(robots_dict)

"""
********************************************************************************
* Packages and Directories
********************************************************************************
"""
description_pkg = "mobile_robot_description"
bringup_pkg = "connected_explorers_bringup"

world_path = os.path.join(get_package_share_directory(bringup_pkg), "worlds", world_name)
rviz_config_file = os.path.join(get_package_share_directory(bringup_pkg), "config", "rviz_config.rviz")
urdf_path = os.path.join(get_package_share_directory(description_pkg), "URDF", "MineMapper.urdf.xacro")
model_path = os.path.join(get_package_share_directory("connected_explorers_bringup"), "models")
gazebo_config_path = os.path.join(get_package_share_directory(bringup_pkg), "config", "gazebo_bridge.yaml")

os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + ":" + model_path

map_file = os.path.join(get_package_share_directory(bringup_pkg), "maps/2d", map_name)

def get_robot_nav_yaml_file(robot_name):
    return os.path.join(
        get_package_share_directory("connected_explorers_bringup"),
        "config",
        f"amcl_config_{robot_name}.yaml",
    )

def get_robot_nav_file(robot_name):
    return os.path.join(
        get_package_share_directory("connected_explorers_bringup"),
        "config",
        f"nav2_planner_config_{robot_name}.yaml",
    )

"""
********************************************************************************
* Launch function
********************************************************************************
"""
def generate_launch_description():
    launch_nodes = []

    # ==========================================================================
    # 1. GAZEBO & RVIZ
    # ==========================================================================
    launch_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"{world_path} -r"}.items(),
    )
    launch_nodes.append(launch_gazebo_node)

    gazebo_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": gazebo_config_path}],
        output="screen",
    )
    launch_nodes.append(gazebo_bridge_node)

    launch_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    launch_nodes.append(launch_rviz_node)

    # ==========================================================================
    # 2. GLOBAL MAP SERVER (Isolated Lifecycle)
    # ==========================================================================
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"topic_name": "map"},
            {"frame_id": "map"},
            {"yaml_filename": map_file},
        ],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )
    launch_nodes.append(map_server)
    launch_nodes.append(map_lifecycle_manager)

    # ==========================================================================
    # 3. GLOBAL UTILITY NODES
    # ==========================================================================
    launch_nodes.append(
        Node(
            package="connected_explorers_connections",
            executable="distance_watcher_node",
            parameters=[
                {"use_sim_time": True},
                {"number_of_robots":len(robots)},
                {"robot_name_prefix":"robot"},
                {"is_3d_mode":False},
                {"los_alpha":-6.0},
                {"los_beta":0.5},
                {"distance_alpha":1.0},
                {"distance_beta":6.0},
            ],
        )
    )
    launch_nodes.append(
        Node(
            package="line_viewer",
            executable="RobotsPositionNode",
            name="RobotsPositionNode",
            parameters=[
                {"robots_list": get_all_robot_names(robots)},
                {"reference_frame": "map"},
            ],
        )
    )
    launch_nodes.append(
        Node(
            package="illustrator",
            executable="illustrator_node",
            name="illustrator_node",
            parameters=[
                {"number_of_robots": len(robots)},
                {"robot_name_prefix": "robot"},
                {"reference_frame": "map"},
            ],
        )
    )
    launch_nodes.append(
        Node(
            package="connected_explorers_global_supervisor",
            executable="supervisor_node",
            parameters=[
                {"number_of_robots": len(robots)},
                {"robot_name_prefix": "robot"},
                {"problem_dimension": 2}
            ],
        )
    )

    # ==========================================================================
    # 4. ROBOTS (Parallel Initialization)
    # ==========================================================================
    for i, robot in enumerate(robots):
        name = robot["name"]
        x_pos = robot["x"]
        y_pos = robot["y"]

        # Define strictly the Nav2 nodes for THIS specific robot
        robot_nav_nodes_list = [
            "amcl",
            "planner_server",
            "controller_server",
            "behavior_server",
            "bt_navigator"
        ]

        launch_nodes.append(Node(
            package="connected_explorers_messagery",
            executable="robot_inbox_node",
            parameters=[
                {"robot_id":i+1},
            ]
        )),

        launch_nodes.append(Node(
            package="connected_explorers_messagery",
            executable="robot_outbox_node",
            parameters=[
                {"robot_id":i+1},
            ]
        )),

        launch_nodes.append(Node(
            package="connected_explorers_laplacian_matrix",
            executable="laplacian_matrix_estimator_node",
            parameters=[
                {"number_of_robots":len(robots)},
                {"robot_index":i+1},
                {"is_3d_mode":False}
            ]
        )),

        # Custom Robot Controller Node
        launch_nodes.append(Node(
            package="line_viewer",
            executable="SingleRobotControllerNode",
            name=f"RobotControllerNode_{name}",
            parameters=[
                {"robots_list": get_all_robot_names(robots)},
                {"robot_number": i + 1},
                {"robot_role": robot["function"]},
                {"holonomic": False},
                {"is_3d_mode": False},
                {"epsilon": 0.3},
                {"gamma": 3.0},
                {"control_period": 0.05},
                {"max_vel": 0.5},
                {"real_max_w": 1.5},
                {"l_point": 0.2},
                {"lambda_conn_threshold": 0.9},
                {"conn_desired_mag": 0.95},
                {"slack_weight": 1e8},
                {"collision_detection_dist": 1.5},
                {"collision_safe_dist": 1.0}
            ],
        )),

        robot_group = GroupAction([
            PushRosNamespace(name),

            # State Publisher & Spawner
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="state_publisher",
                output="screen",
                parameters=[{
                    "robot_description": Command(["xacro ", urdf_path, " robot_ns_arg:=", name]),
                    "use_sim_time": True,
                    "frame_prefix": f"{name}/",
                }],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_entity",
                output="screen",
                arguments=["-name", name, "-topic", "robot_description", "-x", x_pos, "-y", y_pos, "-z", "0.1"],
            ),

            # Nav2 Stack Nodes
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    get_robot_nav_yaml_file(name),  # 1. Load the base config first
                    {
                        # 2. Overwrite/Inject the initial pose dynamically
                        "use_sim_time": True,
                        "set_initial_pose": True,
                        "initial_pose.x": float(x_pos),
                        "initial_pose.y": float(y_pos),
                        "initial_pose.z": 0.0,
                        "initial_pose.yaw": 0.0,
                    }
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[get_robot_nav_file(name)],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[get_robot_nav_file(name)],
                remappings=[("cmd_vel", "ideal_cmd_vel")],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[get_robot_nav_file(name)],
                remappings=[("cmd_vel", "ideal_cmd_vel")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[get_robot_nav_file(name)],
            ),

            

            # DEDICATED LIFECYCLE MANAGER FOR THIS ROBOT ONLY
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name=f"lifecycle_manager_{name}",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": robot_nav_nodes_list},
                ],
            ),
        ])


        # Delaying the robot groups slightly just to let Gazebo bridge connect
        launch_nodes.append(TimerAction(period=3.0, actions=[robot_group]))

    launch_nodes.append(Node(
        package="connected_explorers_messagery",
        executable="router_node",
        parameters=[
            {"number_of_robots":len(robots)}
        ]
    ))

    return LaunchDescription(launch_nodes)
