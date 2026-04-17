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
MAP_FILE = "untitled.binvox.bt"


"""*****************************************************************************
* Data
*****************************************************************************"""
robots = [
    {"name":"robot1","x":-2.5,"y":2.5,"z":5.0,"function":"task"},
    {"name":"robot2","x":-2.5,"y":-2.5,"z":5.0,"function":"conn"},
    {"name":"robot3","x":2.5,"y":2.5,"z":5.0,"function":"conn"},
    {"name":"robot4","x":2.5,"y":-2.5,"z":5.0,"function":"conn"},
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
            "use_sim_time": USE_SIM_TIME
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

    for robot in robots:
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
        package="connected_explorers_connections",
        executable="distance_watcher_node",
        parameters=[
            {"number_of_robots":len(robots)},
            {"robot_name_prefix":"robot"}
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
        ]
    )
    launch_nodes.append(supervisor_node)


    for i,robot in enumerate(robots):
        r_controller_node = Node(
            package="line_viewer",
            executable="SingleRobotControllerNode",
            name=f"RobotControllerNode_{robot['name']}",
            parameters=[
                {"robots_list":get_all_robot_names(robots)},
                {"robot_number":i+1},
                {"robot_role":robot["function"]}
            ]
        )
        launch_nodes.append(r_controller_node)


    return LaunchDescription(launch_nodes)