'''
********************************************************************************
* Multi navigation Launch file
    - This launch file is used to generate multiple robots and control them
********************************************************************************
'''

import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
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


# Scenario 01:

# robots = [
#     {"name":"robot1","x":"-6","y":"-1","function":"task"},
#     {"name":"robot2","x":"-6","y":"1","function":"task"},
#     {"name":"robot3","x":"-7","y":"0","function":"conn"},
# ]
# world_name = 'empty.world'
# map_name = 'empty_world.yaml'


# Scenario 02:

# robots = [
#     {"name":"robot1","x":"-0","y":"-2","function":"task"},
#     {"name":"robot2","x":"-2","y":"1","function":"conn"},
#     {"name":"robot3","x":"-3","y":"-6","function":"conn"},
# ]
# world_name = 'proj_world.world'
# map_name = 'my_map.yaml'

# Scenario 03:

robots = [
    {"name":"robot1","x":"-6","y":"7.5","function":"task"},
    {"name":"robot2","x":"-6","y":"8.5","function":"task"},
    {"name":"robot3","x":"-7","y":"7.5","function":"conn"},
    {"name":"robot4","x":"-7","y":"8.5","function":"conn"},
]
world_name = 'proj_world.world'
map_name = 'my_map.yaml'

# # Scenario 04:

# robots = [
#     {"name":"robot1","x":"-6","y":"7.5","function":"task"},
#     {"name":"robot2","x":"-6","y":"8.5","function":"task"},
#     {"name":"robot3","x":"-7","y":"7.5","function":"conn"},
#     {"name":"robot4","x":"-7","y":"8.5","function":"conn"},
# ]
# world_name = 'proj_world.world'
# map_name = 'my_map.yaml'

lifecycle_managed_nodes = ["map_server"]

def get_all_robot_names(robots):
    return [robot["name"] for robot in robots ]

def get_robots_functions(robots):
    robots_dict = {robot["name"]:robot["function"] for robot in robots}
    return json.dumps(robots_dict)



'''
********************************************************************************
* Packages information
********************************************************************************
'''
description_pkg = 'mobile_robot_description'
bringup_pkg = 'connected_explorers_bringup'

'''
********************************************************************************
* Files directory
********************************************************************************
'''
world_path = os.path.join(
    # get_package_share_directory(bringup_pkg), 'worlds', 'simpler.world'
    get_package_share_directory(bringup_pkg), 'worlds', world_name
    )
rviz_config_file = os.path.join(
    get_package_share_directory(bringup_pkg), 'config', 'rviz_config.rviz'
    )
urdf_path = os.path.join(
    get_package_share_directory(description_pkg), 'URDF', 'MineMapper.urdf.xacro'
    )

model_path = os.path.join(
    get_package_share_directory('connected_explorers_bringup'),
    'models'
)
gazebo_config_path = os.path.join(
    get_package_share_directory(bringup_pkg), 'config', 'gazebo_bridge.yaml'
    )

os.environ['GZ_SIM_RESOURCE_PATH'] = (
    os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + model_path
)



nav_config_path = os.path.join(
    get_package_share_directory(bringup_pkg), 'config', 'nav2_params.yaml'
)

nav2_bringup_dir = get_package_share_directory('nav2_bringup')

bringup_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

map_file = os.path.join(
    # get_package_share_directory(bringup_pkg), 'maps', 'simpler_map.yaml'
    get_package_share_directory(bringup_pkg), 'maps', map_name   
)

'''
------------------------------------------
'''

def get_robot_nav_yaml_file(robot_name):
    return os.path.join(get_package_share_directory(
        "connected_explorers_bringup"), "config", f"amcl_config_{robot_name}.yaml"
    )

def get_robot_nav_file(robot_name):
    return os.path.join(get_package_share_directory(
        "connected_explorers_bringup"), "config", f"nav2_planner_config_{robot_name}.yaml"
    )

'''
********************************************************************************
* launch function
********************************************************************************
'''
def generate_launch_description():
    launch_nodes = []


    '''
    ****************************************************************************
    * GAZEBO
    ****************************************************************************
    '''

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
        parameters=[{"use_sim_time": True}]
    )
    launch_nodes.append(launch_rviz_node)

    '''
    ****************************************************************************
    * MAP SERVER GLOBAL MINIMALISTA (Apenas map e use_sim_time)
    ****************************************************************************
    '''
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time":True},
            {"topic_name":"map"},
            {"frame_id":"map"},
            {"yaml_filename":map_file}
        ]
    )
    launch_nodes.append(map_server)


    '''
    ****************************************************************************
    * Robots
    ****************************************************************************
    '''
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

        pose_node = Node(
            package="initial_pose_estimator",
            executable="initial_pose_estimator",
            name=f"{name}_initial_pose_estimator", 
            output="screen",
            parameters=[{
                "x": float(x_pos),
                "y": float(y_pos),
                "namespace": name
            }]
        )
        # launch_nodes.append(pose_node)

        amcl_node = Node(
            namespace=name,
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters = [get_robot_nav_yaml_file(name)]
        )
        # launch_nodes.append(amcl_node)
    
        planner_node = Node(
            namespace=name,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[get_robot_nav_file(name)] 
        )
        # launch_nodes.append(planner_node)

        controller_node = Node(
            namespace=name,
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[get_robot_nav_file(name)], # O YAML que acabamos de editar
            remappings=[
                ('cmd_vel', 'ideal_cmd_vel')
            ]
        )

        behavior_node = Node(
            namespace=name,
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[get_robot_nav_file(name)],
            remappings=[('cmd_vel', 'ideal_cmd_vel')] # Importante!
        )


        bt_navigator_node = Node(
            namespace=name,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[get_robot_nav_file(name)]
        )


        lifecycle_managed_nodes.append(f"{name}/amcl")
        lifecycle_managed_nodes.append(f"{name}/planner_server")
        lifecycle_managed_nodes.append(f"{name}/controller_server")
        lifecycle_managed_nodes.append(f"{name}/behavior_server")
        lifecycle_managed_nodes.append(f"{name}/bt_navigator")

        delayed_nav_nodes = TimerAction(
            period=10.0, 
            actions=[amcl_node, planner_node, controller_node,behavior_node,bt_navigator_node]
        )

        delayed_pose_node = TimerAction(
            period=20.0, 
            actions=[pose_node]
        )
        
        launch_nodes.append(delayed_nav_nodes)
        launch_nodes.append(delayed_pose_node)
        
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters = [
            {"use_sim_time":True},
            {"autostart":True},
            {"bond_timeout":0.0},
            {"node_names":lifecycle_managed_nodes},
        ]
    )
    delayed_lifecycle_manager = TimerAction(
        period=15.0, 
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

    # marker_node = Node(
    #     package="line_viewer",
    #     executable="SightMarkerNode",
    #     name="SightMarkerNode",
    #     parameters=[
    #         {"robots_list":get_all_robot_names(robots)},
    #         {"reference_frame":"map"},
    #         {"publisher_node_name":"visualization_marker"}

    #     ]
    # )
    # launch_nodes.append(marker_node)

    



    # marker_node = Node(
    #     package="line_viewer",
    #     executable="RobotsControllerNode",
    #     name="RobotsControllerNode",
    #     parameters=[
    #         {"robots_list":get_all_robot_names(robots)},
    #         {'robots_function_map': get_robots_functions(robots)}
    #     ]
    # )
    # launch_nodes.append(marker_node)

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