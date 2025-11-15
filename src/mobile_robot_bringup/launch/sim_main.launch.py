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
    {'name': 'robot1', 'x': 0.0, 'y': 0.0},
    {'name': 'robot2', 'x': 4.0, 'y': 0.0},
    {'name': 'robot3', 'x': 2.0, 'y': 3.0},
    {'name': 'robot4', 'x': -1.0, 'y': 3.0},
    {'name': 'robot5', 'x': 5.0, 'y': 3.0},
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
    get_package_share_directory(bringup_pkg), 'config', 'rviz_visualization_config.rviz'
    )
urdf_path = os.path.join(
    get_package_share_directory(description_pkg), 'URDF', 'MineMapper.urdf.xacro'
    )
gazebo_config_path = os.path.join(
    get_package_share_directory(bringup_pkg), 'config', 'gazebo_bridge.yaml'
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
    * Software launch
    ****************************************************************************
    '''

    launch_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_path} -r'}.items()
    )
    launch_nodes.append(launch_gazebo_node)

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
    * Robots creation
    ****************************************************************************
    '''

    for robot in robots:
        name = robot['name']
        x_pos = str(robot['x'])
        y_pos = str(robot['y'])

        
        robot_group = GroupAction(
            actions=[
                PushRosNamespace(name),

                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='state_publisher',
                    output='screen',
                    parameters=[{
                        'robot_description': Command(['xacro ', urdf_path, ' robot_ns:=', name]),
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
                ),

                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge',
                    arguments=[
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',                        
                        f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',                        
                        f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',                        
                        f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        
                        f'/world/empty/model/{name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
                    ],
                    remappings=[
                        (f'/model/{name}/odometry', 'odom'),
                        (f'/{name}/scan', 'scan'),
                        (f'/model/{name}/cmd_vel', 'cmd_vel'),

                        (f'/world/empty/model/{name}/joint_state', 'joint_states')
                    ]
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    namespace=name,
                    name='world_to_base',
                    arguments=[str(robot['x']), str(robot['y']), '0', '0', '0', '0', 'world', f'{name}/base_footprint'],
                    output='screen'
                )
            ]
        )
        launch_nodes.append(robot_group)
    

    return LaunchDescription(launch_nodes)

