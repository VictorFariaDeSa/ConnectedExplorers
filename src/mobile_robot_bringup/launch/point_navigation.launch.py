import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
import json

BRINGUP_PACKAGE = 'mobile_robot_bringup'
MAP_NAME = 'my_map.yaml'
RVIZ_CONFIG_FILE = "point_rviz_config.rviz"
USE_SIM_TIME = False





# SCORE CALCULATIONS
SIGHT_SCORE_OFFSET = 0.5
SIGHT_SCORE_SCALE = -6.0

DISTANCE_SCORE_OFFSET = 6.0
DISTANCE_SCORE_SCALE = 1.0




robots = [
    {"name":"robot1","x":-6.0,"y":7.5,"function":"task"},
    {"name":"robot2","x":-6.0,"y":8.5,"function":"task"},
    {"name":"robot3","x":-7.0,"y":7.5,"function":"conn"},
    {"name":"robot4","x":-7.0,"y":8.5,"function":"conn"},
]



def get_all_robot_names(robots):
    return [robot["name"] for robot in robots ]

def get_robots_functions(robots):
    robots_dict = {robot["name"]:robot["function"] for robot in robots}
    return json.dumps(robots_dict)

map_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'maps', MAP_NAME)
rviz_config_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'config', RVIZ_CONFIG_FILE)

def get_robot_nav_file(robot_name):
    return os.path.join(get_package_share_directory(
        BRINGUP_PACKAGE), "config", f"point_config_{robot_name}.yaml"
    )

nav_file_robot = get_robot_nav_file("robot1")

# Nós gerenciados pelo Lifecycle (incluindo os do robot1 com namespace)
lifecycle_managed_nodes = [
    "map_server", 
]

def generate_launch_description():
    launch_nodes = []

    # 1. RViz
    launch_nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{"use_sim_time": USE_SIM_TIME}]
    ))

    # 2. Map Server (Global)
    launch_nodes.append(Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": USE_SIM_TIME}, {"yaml_filename": map_file}]
    ))

    # 3. Robôs (Simulador Customizado)
    for robot in robots:
        name = robot["name"]
        x_pos = robot["x"]
        y_pos = robot["y"]
        nav_file_robot = get_robot_nav_file(name)
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
                "xPos": robot["x"],      # Agora pegando a posição real da lista
                "yPos": robot["y"], 
                "yaw": 0.0, 
                "task": robot["function"],
                "use_sim_time": USE_SIM_TIME  # Crucial para as TFs aparecerem
            }]
        ))

        
        # 4. Nav2 Nodes para o Robot1
        planner_node = Node(
            namespace=name,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav_file_robot]
        )

        controller_node = Node(
            namespace=name,
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav_file_robot],
            remappings=[
                ('cmd_vel', 'ideal_cmd_vel')
            ]
        )

        bt_navigator_node = Node(
            namespace=name,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav_file_robot]
        )

        behavior_node = Node(
                namespace=name,
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav_file_robot],
                remappings=[
                    ('cmd_vel', 'ideal_cmd_vel')
                ]
            )


        lifecycle_managed_nodes.append(f"{name}/planner_server")
        lifecycle_managed_nodes.append(f"{name}/controller_server")
        lifecycle_managed_nodes.append(f"{name}/behavior_server")
        lifecycle_managed_nodes.append(f"{name}/bt_navigator")

        delayed_nav_nodes = TimerAction(
            period=1.0, 
            actions=[planner_node, controller_node,behavior_node,bt_navigator_node]
        )
        launch_nodes.append(delayed_nav_nodes)

    # 5. Lifecycle Manager
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": USE_SIM_TIME, 
            "autostart": True, 
            "node_names": lifecycle_managed_nodes,
            "bond_timeout": 0.0
        }]
    )

    delayed_lifecycle_manager = TimerAction(
        period=5.0, 
        actions=[lifecycle_manager_node]
    )
    launch_nodes.append(delayed_lifecycle_manager)

    marker_node = Node(
        package="line_viewer",
        executable="RobotsPositionNode",
        name="RobotsPositionNode",
        parameters=[
            {"robots_list":get_all_robot_names(robots)},
            {"reference_frame":"map"}
        ]
    )
    launch_nodes.append(marker_node)

    marker_node = Node(
        package="line_viewer",
        executable="SightMarkerNode",
        name="SightMarkerNode",
        parameters=[
            {"robots_list":get_all_robot_names(robots)},
            {"reference_frame":"map"},
            {"publisher_node_name":"visualization_marker"},
            {'robots_function_map': get_robots_functions(robots)}

        ]
    )
    launch_nodes.append(marker_node)


    marker_node = Node(
        package="line_viewer",
        executable="RobotsMathNode",
        name="RobotsMathNode",
        parameters=[
            {"robots_list":get_all_robot_names(robots)},
            {"sight_score_offset":      SIGHT_SCORE_OFFSET},
            {"sight_score_scale":       SIGHT_SCORE_SCALE},
            {"distance_score_offset":   DISTANCE_SCORE_OFFSET},
            {"distance_score_scale":    DISTANCE_SCORE_SCALE},
            {"laplacian_topic_name":"laplacian_matrix"},

        ]
    )
    launch_nodes.append(marker_node)


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