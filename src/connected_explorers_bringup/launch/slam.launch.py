'''
********************************************************************************
* Slam Launch file
    - This launch file must be used to generate the map
* Slam Launch file
    - This launch file must be used to generate the map
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
    get_package_share_directory(bringup_pkg), 'worlds', 'proj_world.world'
    )
rviz_config_file = os.path.join(
    get_package_share_directory(bringup_pkg), 'slam_config', 'rviz_config.rviz'
    )
urdf_path = os.path.join(
    get_package_share_directory(description_pkg), 'URDF', 'MineMapper.urdf.xacro'
    )
gazebo_config_path = os.path.join(
    get_package_share_directory(bringup_pkg), 'slam_config', 'gazebo_bridge.yaml'
    )
slam_toolbox_params_file = os.path.join(
    get_package_share_directory(bringup_pkg), 'slam_config', 'mapper_params_online_async.yaml'
    )
model_path = os.path.join(
    get_package_share_directory('connected_explorers_bringup'),
    'models'
)

os.environ['GZ_SIM_RESOURCE_PATH'] = (
    os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + model_path
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
        output='screen'
    )
    launch_nodes.append(launch_rviz_node)


    '''
    ****************************************************************************
    * Robot creation
    ****************************************************************************
    '''

    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True,
        }]
    )
    launch_nodes.append(state_publisher_node)

    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[       
            '-name', 'MineMapper',                      
            '-topic', 'robot_description',  
            '-x', '0',                    
            '-y', '-3',                    
            '-z', '0.1'                     
        ]
    )
    launch_nodes.append(robot_spawn_node)

    


    '''
    ****************************************************************************
    * SLAM
    ****************************************************************************
    '''
    launch_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_sync_launch.py' # Usa o launch file que você testou
            )
        ]),
        launch_arguments={
            # Passa o caminho completo do seu arquivo de parâmetros para o argumento 'params_file'
            'params_file': slam_toolbox_params_file, 
            'use_sim_time': 'true' # Garante a sincronização com o Gazebo
        }.items()
    )
    launch_nodes.append(launch_slam_toolbox)   






    return LaunchDescription(launch_nodes)

