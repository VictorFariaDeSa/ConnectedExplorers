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
    * MAP SERVER GLOBAL MINIMALISTA (Apenas map e use_sim_time)
    ****************************************************************************
    '''
    launch_global_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': "True",
            'params_file': nav_config_path,
        }.items()
    )
    launch_nodes.append(launch_global_map_server)

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


    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        output='screen'
    )
    launch_nodes.append(gazebo_bridge_node)

    return LaunchDescription(launch_nodes)