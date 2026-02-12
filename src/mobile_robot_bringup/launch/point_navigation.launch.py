import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction

BRINGUP_PACKAGE = 'mobile_robot_bringup'
MAP_NAME = 'my_map.yaml'
RVIZ_CONFIG_FILE = "point_rviz_config.rviz"
USE_SIM_TIME = False

robots = [
    {"name":"robot1","x":-6.0,"y":7.5,"function":"task"},
    # {"name":"robot2","x":-6.0,"y":8.5,"function":"task"},
    # {"name":"robot3","x":-7.0,"y":7.5,"function":"conn"},
    # {"name":"robot4","x":-7.0,"y":8.5,"function":"conn"},
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
        parameters=[nav_file_robot1]
    )

    controller_node = Node(
        namespace=name,
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_file_robot1],
    )

    bt_navigator_node = Node(
        namespace=name,
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav_file_robot1]
    )

    behavior_node = Node(
            namespace=name,
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav_file_robot1],
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

    return LaunchDescription(launch_nodes)