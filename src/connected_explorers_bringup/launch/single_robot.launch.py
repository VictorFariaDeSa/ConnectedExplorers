'''*****************************************************************************
* Imports
*****************************************************************************'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

'''*****************************************************************************
* Defines
*****************************************************************************'''
DEFAULT_NAMESPACE_VALUE = "robot0"



def generate_launch_description():
    launch_nodes = []
    namespace_arg = DeclareLaunchArgument('robot_namespace', default_value='robot_0')
    namespace = LaunchConfiguration('robot_namespace')

    # This pushes ALL following nodes into the namespace
    push_ns = PushRosNamespace(namespace)

    return LaunchDescription(launch_nodes)