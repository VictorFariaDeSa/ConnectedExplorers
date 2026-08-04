"""*****************************************************************************
* Imports
*****************************************************************************"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

"""*****************************************************************************
* Defines
*****************************************************************************"""

USE_SIM_TIME = False
MAP_FRAME_ID = "map"


# packages ---
BRINGUP_PACKAGE = 'connected_explorers_bringup'

# files ---
RVIZ_CONFIG_FILE = "point3d_config.rviz"


# scenario 01 ---
MAP_FILE = "flat.binvox.bt"
# # scenario 02 ---
MAP_FILE = "wall.binvox.bt"
# # # # scenario 04 ---
MAP_FILE = "tunnel.binvox.bt"
# # # # scenario 04 ---
# # # MAP_FILE = "tunnel.binvox.bt"
# # # # scenario 04 ---
MAP_FILE = "untitled.binvox.bt"


"""*****************************************************************************
* Data
*****************************************************************************"""
# scenario 01 ---
robots = [
    {"name": "robot1", "x": -3.0, "y": 0.0, "z": 5.0, "function": "task"},
    {"name": "robot2", "x": 3.0, "y": 0.0, "z": 5.2, "function": "base"}
]

# scenario 02 ---
robots = [
    {"name": "robot1", "x": 3.0, "y": -2.0, "z": 2.0, "function": "task"},
    {"name": "robot2", "x": 3.0, "y": 0.0, "z": 2.0, "function": "conn"},
    {"name": "robot3", "x": 3.0, "y": 2.0, "z": 2.0, "function": "base"}
]

# # scenario 04 ---
robots = [
    {"name": "robot1", "x": 0.0, "y": 0.0,  "z": 5.0, "function": "task"},
    {"name": "robot2", "x": -3.0, "y": 0.0,  "z": 2.0, "function": "conn"},
    {"name": "robot3", "x": 1.5, "y": 2.6, "z": 2.0, "function": "conn"},
    {"name": "robot4", "x": 1.5, "y": -2.6,"z": 2.0, "function": "conn"},
    {"name": "robot5", "x": 0.0, "y": 0.0,  "z": 2.0, "function": "base"}
]


robots = [
    {"name": "robot1", "x": -2.5, "y": 2.5, "z": 5.0, "function": "task"},
    {"name": "robot2", "x": -2.5, "y": -2.5, "z": 5.0, "function": "conn"},
    {"name": "robot3", "x": 2.5, "y": 2.5, "z": 5.0, "function": "conn"},
    {"name": "robot4", "x": 2.5, "y": -2.5, "z": 5.0, "function": "conn"},
    {"name": "robot5", "x": 0.0, "y": 0.0, "z": 2.0, "function": "base"},
]

# TODO DELETE
def get_all_robot_names(robots):
    return [robot["name"] for robot in robots ]

"""*****************************************************************************
* Config Files
*****************************************************************************"""
rviz_config_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'config/rviz', RVIZ_CONFIG_FILE)
map_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'maps/3d', MAP_FILE)


"""*****************************************************************************
* launch
*****************************************************************************"""
def generate_launch_description():
    launch_nodes = []

    # rviz2 ---
    launch_nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{"use_sim_time": USE_SIM_TIME}]
    ))

    # map publisher ---
    launch_nodes.append(Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            "octomap_path": map_file,
            "frame_id": MAP_FRAME_ID,
            "use_sim_time": USE_SIM_TIME,
            "height_map": False,
            "color.r": 0.8,
            "color.g": 0.8,
            "color.b": 0.8,
            "color.a": 1.0,
        }]
    ))

    # map tf ---
    launch_nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_world_to_map',
        arguments=[
            '--x', '0', 
            '--y', '0', 
            '--z', '0', 
            '--roll', '0', 
            '--pitch', '0', 
            '--yaw', '0', 
            '--frame-id', 'map', 
            '--child-frame-id', 'world'
        ]
    ))

    for i,robot in enumerate(robots):
        if robot["function"] != "base":
            r_controller_node = Node(
                package="line_viewer",
                executable="SingleRobotControllerNode",
                name=f"RobotControllerNode_{robot['name']}",
                parameters=[
                    {"robots_list":get_all_robot_names(robots)},
                    {"robot_number":i+1},
                    {"is_3d_mode":True},
                    {"holonomic":True},
                    {"robot_role":robot["function"]}

                ]
            )
            launch_nodes.append(r_controller_node)

        name = robot["name"]
        launch_nodes.append(Node(
            package="line_viewer",
            executable="PointRobotNode",
            name="PointRobotNode",
            namespace=name,
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ],
            parameters=[{
                "xPos": robot["x"],
                "yPos": robot["y"], 
                "zPos": robot["z"], 
                "yaw": 0.0, 
                "task": robot["function"],
                "use_sim_time": USE_SIM_TIME,
                "frame_id":"world"
            }]
        ))

        launch_nodes.append(Node(
            package="connected_explorers_messagery",
            executable="robot_inbox_node",
            parameters=[
                {"robot_id":i+1},
            ]
        ))

        launch_nodes.append(Node(
            package="connected_explorers_messagery",
            executable="robot_outbox_node",
            parameters=[
                {"robot_id":i+1},
            ]
        ))

        launch_nodes.append(Node(
            package="connected_explorers_laplacian_matrix",
            executable="laplacian_matrix_estimator_node",
            parameters=[
                {"number_of_robots":len(robots)},
                {"robot_index":i+1},
                {"is_3d_mode":True}
            ]
        ))

    launch_nodes.append(Node(
        package="connected_explorers_messagery",
        executable="router_node",
        parameters=[
            {"number_of_robots":len(robots)}
        ]
    ))

    launch_nodes.append(Node(
        package="connected_explorers_connections",
        executable="distance_watcher_node",
        parameters=[
            {"number_of_robots":len(robots)},
            {"robot_name_prefix":"robot"},
            {"is_3d_mode":True},
            {"los_alpha":-6.0},
            {"los_beta":0.5},
            {"distance_alpha":1.0},
            {"distance_beta":6.0},
        ]
    ))

    pos_node = Node(
        package="line_viewer",
        executable="RobotsPositionNode",
        name="RobotsPositionNode",
        parameters=[
            {"robots_list":get_all_robot_names(robots)},
            {"reference_frame":"map"}
        ]
    )
    launch_nodes.append(pos_node)


    marker_node = Node(
        package="illustrator",
        executable="illustrator_node",
        name="illustrator_node",
        parameters=[
            {"number_of_robots":len(robots)},
            {"robot_name_prefix":"robot"},
            {"reference_frame":"map"},
        ]
    )
    launch_nodes.append(marker_node)

    supervisor_node = Node(
        package="connected_explorers_global_supervisor",
        executable="supervisor_node",
        parameters=[
            {"number_of_robots":len(robots)},
            {"robot_name_prefix":"robot"},
            {"problem_dimension": 3}
        ]
    )
    launch_nodes.append(supervisor_node)




    data_node = Node(
        package="line_viewer",
        executable="DataRecorderNode",
        name="DataRecorderNode",
        parameters=[
            {"robots_list":get_all_robot_names(robots)}
        ]
    )
    launch_nodes.append(data_node)



    return LaunchDescription(launch_nodes)