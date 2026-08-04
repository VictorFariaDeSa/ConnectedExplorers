import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
import json

BRINGUP_PACKAGE = 'connected_explorers_bringup'
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

map_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'maps/2d', MAP_NAME)
rviz_config_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'config/rviz', RVIZ_CONFIG_FILE)

def get_robot_nav_file(robot_name):
    return os.path.join(get_package_share_directory(
        BRINGUP_PACKAGE), "config", f"point_config_{robot_name}.yaml"
    )

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
    for i,robot in enumerate(robots):
        r_controller_node = Node(
            package="line_viewer",
            executable="SingleRobotControllerNode",
            name=f"RobotControllerNode_{robot['name']}",
            parameters=[
                {"robots_list":get_all_robot_names(robots)},
                {"robot_number":i+1},
                {"is_3d_mode":False},
                {"holonomic":True},
                {"robot_role":robot["function"]}
            ]
        )
        launch_nodes.append(r_controller_node)


        name = robot["name"]
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
                {"is_3d_mode":False}
            ]
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
            {"is_3d_mode":False},
            {"los_alpha":-6.0},
            {"los_beta":0.5},
            {"distance_alpha":1.0},
            {"distance_beta":6.0},
        ]
    ))


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
        package="illustrator",
        executable="illustrator_node",
        name="illustrator_node",
        parameters=[
            {"number_of_robots":len(get_all_robot_names(robots))},
            {"robot_name_prefix":"robot"},
            {"reference_frame":"map"},
        ]
    )
    launch_nodes.append(marker_node)

    supervisor_node = Node(
        package="connected_explorers_global_supervisor",
        executable="supervisor_node",
        parameters=[
            {"number_of_robots":len(get_all_robot_names(robots))},
            {"robot_name_prefix":"robot"},
            {"problem_dimension": 2}
        ]
    )
    launch_nodes.append(supervisor_node)



    return LaunchDescription(launch_nodes)