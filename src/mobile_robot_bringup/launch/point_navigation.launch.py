import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

BRINGUP_PACKAGE = 'mobile_robot_bringup'
MAP_NAME = 'my_map.yaml'
RVIZ_CONFIG_FILE = "point_rviz_config.rviz"
USE_SIM_TIME = False

robots = [
    {"name":"robot1","x":-6.0,"y":7.5,"function":"task"},
    {"name":"robot2","x":-6.0,"y":8.5,"function":"task"},
    {"name":"robot3","x":-7.0,"y":7.5,"function":"conn"},
    {"name":"robot4","x":-7.0,"y":8.5,"function":"conn"},
]

map_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'maps', MAP_NAME)
rviz_config_file = os.path.join(get_package_share_directory(BRINGUP_PACKAGE), 'config', RVIZ_CONFIG_FILE)

def get_robot_nav_file(robot_name):
    return os.path.join(get_package_share_directory(
        BRINGUP_PACKAGE), "config", f"point_config_{robot_name}.yaml"
    )

nav_file_robot1 = get_robot_nav_file("robot1")

# Nós gerenciados pelo Lifecycle (incluindo os do robot1 com namespace)
lifecycle_managed_nodes = [
    "map_server", 
    "robot1/planner_server", 
    "robot1/controller_server", 
    "robot1/bt_navigator"
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
        launch_nodes.append(Node(
            package="line_viewer",
            executable="PointRobotNode",
            name="PointRobotNode",
            namespace=robot["name"],
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
        # O static_transform_publisher FOI REMOVIDO para não conflitar com o nó Python

    # 4. Nav2 Nodes para o Robot1
    launch_nodes.append(Node(
        namespace="robot1",
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav_file_robot1]
    ))

    launch_nodes.append(Node(
        namespace="robot1",
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav_file_robot1]
    ))

    launch_nodes.append(Node(
        namespace="robot1",
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav_file_robot1]
    ))

    # 5. Lifecycle Manager
    launch_nodes.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "use_sim_time": USE_SIM_TIME, 
            "autostart": True, 
            "node_names": lifecycle_managed_nodes
        }]
    ))

    return LaunchDescription(launch_nodes)