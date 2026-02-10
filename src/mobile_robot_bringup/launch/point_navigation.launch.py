


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

BRINGUP_PACKAGE = 'mobile_robot_bringup'
MAP_NAME = 'my_map.yaml'
RVIZ_CONFIG_FILE = "point_rviz_config.rviz"
USE_SIM_TIME = True







robots = [
    {"name":"robot1","x":-6.0,"y":7.5,"function":"task"},
    {"name":"robot2","x":-6.0,"y":8.5,"function":"task"},
    {"name":"robot3","x":-7.0,"y":7.5,"function":"conn"},
    {"name":"robot4","x":-7.0,"y":8.5,"function":"conn"},
]





map_file = os.path.join(
    get_package_share_directory(BRINGUP_PACKAGE), 'maps', MAP_NAME   
)
rviz_config_file = os.path.join(
    get_package_share_directory(BRINGUP_PACKAGE), 'config', RVIZ_CONFIG_FILE
)


lifecycle_managed_nodes = ["map_server"]


def generate_launch_description():
    launch_nodes = []
    '''
    ****************************************************************************
    * RVIZ
    ****************************************************************************
    '''
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{"use_sim_time": USE_SIM_TIME}]
    )
    launch_nodes.append(launch_rviz_node)


    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time":USE_SIM_TIME},
            {"topic_name":"map"},
            {"frame_id":"map"},
            {"yaml_filename":map_file}
        ]
    )
    launch_nodes.append(map_server)

    for robot in robots:
        point_node = Node(
            package="line_viewer",
            executable="PointRobotNode",
            name="PointRobotNode",
            namespace=robot["name"],
            parameters=[
                {"xPos": 0.0},
                {"yPos": 0.0},
                {"yaw": 0.0},
                {"task": robot["function"]},

            ]
        )
        launch_nodes.append(point_node)
        
        launch_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'link_map_to_{robot["name"]}_odom',
                arguments=[str(robot["x"]), str(robot["y"]), '0', '0', '0', '0', 'map', f'{robot["name"]}/odom']
            )
        )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters = [
            {"use_sim_time":USE_SIM_TIME},
            {"autostart":True},
            {"bond_timeout":0.0},
            {"node_names":lifecycle_managed_nodes},
        ]
    )
    launch_nodes.append(lifecycle_manager_node)





    return LaunchDescription(launch_nodes)