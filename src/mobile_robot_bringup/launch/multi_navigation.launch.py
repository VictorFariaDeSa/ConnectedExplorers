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

'''
********************************************************************************
* User defined params
********************************************************************************
'''
robots = [
    {"name":"robot1","x":"1","y":"-3"},
    {"name":"robot2","x":"-2","y":"-1"}
]

'''
********************************************************************************
* Packages information
********************************************************************************
'''
description_pkg = 'mobile_robot_description'
bringup_pkg = 'mobile_robot_bringup'

'''
********************************************************************************
* Files directory
********************************************************************************
'''
world_path = os.path.join(
    get_package_share_directory(bringup_pkg), 'worlds', 'proj_world.world'
    )
rviz_config_file = os.path.join(
    get_package_share_directory(bringup_pkg), 'config', 'rviz_config.rviz'
    )
urdf_path = os.path.join(
    get_package_share_directory(description_pkg), 'URDF', 'MineMapper.urdf.xacro'
    )

model_path = os.path.join(
    get_package_share_directory('mobile_robot_bringup'),
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
    get_package_share_directory(bringup_pkg), 'maps', 'my_map.yaml'
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
        output='screen'
    )
    launch_nodes.append(launch_rviz_node)


    '''
    ****************************************************************************
    * NAV2
    ****************************************************************************
    '''
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True,
            'yaml_filename': map_file}
        ]
    )
    launch_nodes.append(map_server_node)
    
    nav_lifecicle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']}
        ]
    )
    launch_nodes.append(nav_lifecicle_manager_node)

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

        # robot_state_publisher — REMAPPED tf topics (do NOT use frame_prefix)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path,' robot_ns_arg:=', name]),
                'use_sim_time': True,
            }],
            # <<-- IMPORTANT: remap to relative topics so TF lives in /<namespace>/tf
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
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
        ),

        # AMCL — REMAPPED tf topics and correct global_frame
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # keep frame names simple: base_link / odom; AMCL uses global_frame_id 'map'
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',            # make sure AMCL references the global 'map'
                'scan_topic': 'scan',                # relative topic -> /robotX/scan
                'tf_broadcast': True,
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),

        # per-robot lifecycle manager (if you keep it)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=f'lifecycle_manager_localization_{name}',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['amcl'],
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            ),
        ])
        launch_nodes.append(group)

        


    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        output='screen'
    )
    launch_nodes.append(gazebo_bridge_node)

    return LaunchDescription(launch_nodes)