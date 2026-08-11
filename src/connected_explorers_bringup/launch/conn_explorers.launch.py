#===============================================================================
# IMPORTS
#===============================================================================
import os
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import tomllib
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from enum import Enum


#===============================================================================
# DEFINES
#===============================================================================
BRINGUP_PKG_NAME = "connected_explorers_bringup"
DESCRIPTION_PKG_NAME = "mobile_robot_description"


RVIZ_CONFIG_DIRECTORY = "config/rviz"
GAZEBO_CONFIG_DIRECTORY = "config/gazebo"
GAZEBO_MODELS_DIRECTORY = "maps/assets"
ROBOT_URDF_DIRECTORY = "URDF"
ROBOT_URDF_MODEL_FILE = "MineMapper.urdf.xacro"

CURRENT_SCENARIO = "gz_2d_empty"



GLOBAL_TF_REMAPPINGS = [
    ('/tf', '/tf'),
    ('/tf_static', '/tf_static')
]

NAV_REMAPPINGS = GLOBAL_TF_REMAPPINGS + [
    ('map', '/map')
]


#===============================================================================
# Enums
#===============================================================================
class Engines(Enum):
    GAZEBO = "gazebo"
    POINT = "point_integrator"

class Localization(Enum):
    AMCL = "amcl"
    GROUND_TRUTH = "ground_truth"

class Navigation(Enum):
    NAV2 = "nav2"
    TELEOP = "teleop"

class Kinematics(Enum):
    DIFFERENTIAL = "differential"
    HOLONOMIC = "holonomic"

class Dimensions(Enum):
    DIMENSION_2 = "2D"
    DIMENSION_3 = "3D"

#===============================================================================
# HELPER FUNCTIONS
#===============================================================================
def get_robot_state_publisher_node(robot_name: str, use_sim_time: bool):
    urdf_path = os.path.join(
        get_package_share_directory(DESCRIPTION_PKG_NAME), 
        ROBOT_URDF_DIRECTORY, 
        ROBOT_URDF_MODEL_FILE
    ) 

    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_path, " robot_ns_arg:=", robot_name]),
            "use_sim_time": use_sim_time,
            "frame_prefix": f"{robot_name}/",
        }],
        remappings=GLOBAL_TF_REMAPPINGS
    )




# --- gazebo ---
def get_start_gazebo_nodes(
        world_name:str,
        use_sim_time:bool,
        gz_bridge_config_file_name:str
        ):
    # env configurations
    model_path = os.path.join(
        get_package_share_directory(BRINGUP_PKG_NAME),
        GAZEBO_MODELS_DIRECTORY
        )
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + ":" + model_path     

    world_file = os.path.join(
        get_package_share_directory(BRINGUP_PKG_NAME), 
        "maps", 
        f"{world_name}/{world_name}.world"
    )
    gazebo_bridge_config_file = os.path.join(
        get_package_share_directory(BRINGUP_PKG_NAME), 
        GAZEBO_CONFIG_DIRECTORY, 
        gz_bridge_config_file_name
    )


    # nodes
    gazebo_nodes = []

    gazebo_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"), 
                    "launch", 
                    "gz_sim.launch.py"
                )
            ),
            launch_arguments={
                "gz_args": f'{world_file} -r'
                }.items(),
        )
    )

    gazebo_nodes.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"config_file": gazebo_bridge_config_file}
                ],
            output="screen",
        )     
    )

    return gazebo_nodes

def spawn_robot_in_gazebo_world(
        robot_name:str,
        world_name:str,
        x:float,
        y:float,
        z:float
        ):
        return Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{robot_name}",
            output="screen",
            arguments=[
                "-world", world_name,
                "-name", robot_name, 
                "-topic", "robot_description",                        
                "-x", str(x), 
                "-y", str(y), 
                "-z", str(z)
            ]
        )
    
def get_rviz_configuration_file(is_gz: bool, is_3d_mode: bool)->str:
    if is_gz:
        rviz_file_name = "rviz_config.rviz"
    elif is_3d_mode:
        rviz_file_name = "point3f_config.rviz"
    else:
        rviz_file_name = "point_rviz_config.rviz"

    return os.path.join(
        get_package_share_directory(BRINGUP_PKG_NAME),
        "config","rviz",
        rviz_file_name
    )

def get_robot_amcl_yaml_file(robot_name: str):
    return os.path.join(
        get_package_share_directory(BRINGUP_PKG_NAME),
        "config","nav2","amcl",
        f"amcl_config_{robot_name}.yaml",
    )

def get_robot_nav_file(robot_name:str, kinematics: str):
    match kinematics:
        case Kinematics.DIFFERENTIAL:
            folder = "differential"
        case Kinematics.HOLONOMIC:
            folder = "holonomic"
        case _:
            pass


    return os.path.join(
        get_package_share_directory(BRINGUP_PKG_NAME),
        "config","nav2","navigation",folder,
        f"navigation_config_{robot_name}.yaml",
    )


def generate_launch_description():
    #===========================================================================
    # SCENARIO PARSE
    #===========================================================================
    toml_scenario_file = os.path.join(
            get_package_share_directory(BRINGUP_PKG_NAME), 
            "scenarios", 
            f"{CURRENT_SCENARIO}.toml"
            )

    with open(toml_scenario_file, "rb") as f:
        cfg = tomllib.load(f)

    # --- environment variables ---
    env_cfg = cfg["environment"]
    engine = Engines(env_cfg["engine"])
    dimension = Dimensions(env_cfg["dimension"])
    map = env_cfg["map"]
    use_sim_time = env_cfg["use_sim_time"]

    # --- platform variables ---
    plat_cfg = cfg["platform"]
    navigation = Navigation(plat_cfg["navigation"])
    localization = Localization(plat_cfg["localization"])
    kinematics = Kinematics(plat_cfg["kinematics"])

    # --- cbf variables ---
    cbf_cfg = cfg.get("cbf", {})
    los_alpha = cbf_cfg.get("los_alpha", -6.0)
    los_beta = cbf_cfg.get("los_beta", 0.5)
    distance_alpha = cbf_cfg.get("distance_alpha", 1.0)
    distance_beta = cbf_cfg.get("distance_beta", 6.0)

    # --- external files ---
    ext_files = cfg.get("external_files", {})
    gz_bridge_file_name = ext_files.get("gz_bridge_config", None)

    # --- swarm ---
    robots = cfg["robots"]

    
    

    
    
    
    
    
    

    global_nodes = []
    per_robot_nodes = {robot["name"]: [] for robot in robots}
    per_robot_lc_nodes = {robot["name"]: [] for robot in robots}
    # ==========================================================================
    # 0. Visualization
    # ==========================================================================
    rviz_config_file = get_rviz_configuration_file(
            is_3d_mode = dimension == "3D",
            is_gz = engine == "gazebo"
        )
    # --- rviz ---
    global_nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    # ==========================================================================
    # 1. ENGINE
    # ==========================================================================
    match engine:
        case Engines.GAZEBO:
            global_nodes.extend(get_start_gazebo_nodes(
                world_name=map,
                use_sim_time=use_sim_time,
                gz_bridge_config_file_name=gz_bridge_file_name
            ))
            for robot in robots:
                robot_name = robot["name"]
                per_robot_nodes[robot_name].extend([
                    spawn_robot_in_gazebo_world(
                        robot_name=robot["name"],
                        world_name="empty",
                        x=robot["x"],
                        y=robot["y"],
                        z=robot["z"]
                    ),   
                    get_robot_state_publisher_node(
                        robot_name=robot_name,
                        use_sim_time=use_sim_time
                    )
                    
                ])
        case Engines.POINT:
            for robot in robots:
                robot_name = robot["name"]
                per_robot_nodes[robot_name].append(Node(
                    package="line_viewer",
                    executable="PointRobotNode",
                    name="PointRobotNode",
                    remappings=GLOBAL_TF_REMAPPINGS,
                    parameters=[{
                        "xPos": robot["x"],
                        "yPos": robot["y"], 
                        "zPos": robot["z"], 
                        "yaw": 0.0, 
                        "task": robot["function"],
                        "use_sim_time": use_sim_time,
                        "frame_id":"map"
                    }]
                ))
        case _:
            raise ValueError(f"Engine '{engine}' is not available.")

        

    # ==========================================================================
    # 2. LOCALIZATION
    # ==========================================================================
    match localization:
        case Localization.AMCL:
            for robot in robots:
                robot_name = robot["name"]
                per_robot_lc_nodes[robot_name].append("amcl")

                per_robot_nodes[robot_name].append(
                    Node(
                        package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                        parameters=[
                            get_robot_amcl_yaml_file(robot_name),
                            {
                                "use_sim_time": use_sim_time,
                                "set_initial_pose": True, 
                                "initial_pose.x": robot["x"], 
                                "initial_pose.y": robot["y"], 
                                "initial_pose.z": robot["z"], 
                                "initial_pose.yaw": 0.0
                            }
                        ],
                        remappings=NAV_REMAPPINGS
                    )
                )
        case Localization.GROUND_TRUTH:
            pass
        case _:
            pass

    # ==========================================================================
    # 2. NAVIGATION
    # ==========================================================================
    match navigation:
        case Navigation.NAV2:
            occupancy_grid = os.path.join(
                get_package_share_directory(BRINGUP_PKG_NAME), 
                "maps", 
                f"{map}/{map}.yaml"
            )
    
            global_nodes.extend([
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"topic_name": "map"}, #TODO remover hardcoded
                        {"frame_id": "map"}, #TODO remover hardcoded
                        {"yaml_filename": occupancy_grid},
                    ],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_map",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"autostart": True}, #TODO remover hardcoded
                        {"node_names": ["map_server"]}, #TODO remover hardcoded
                    ],
                )
            ])


            for robot in robots:
                robot_name = robot["name"]
                common_params = [get_robot_nav_file(robot_name, kinematics), {"use_sim_time": use_sim_time}]
                cmd_vel_remapping = [("cmd_vel", "ideal_cmd_vel")] + NAV_REMAPPINGS
                
                per_robot_lc_nodes[robot_name].extend([
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator"
                ])
                
                # Add to this robot's node basket
                per_robot_nodes[robot_name].extend([
                    Node(
                        package="nav2_planner", 
                        executable="planner_server", 
                        name="planner_server", 
                        output="screen", 
                        parameters=common_params, 
                        remappings=NAV_REMAPPINGS
                    ),
                    Node(
                        package="nav2_controller", 
                        executable="controller_server", 
                        name="controller_server", 
                        output="screen", 
                        parameters=common_params, 
                        remappings=cmd_vel_remapping
                    ),
                    Node(
                        package="nav2_behaviors", 
                        executable="behavior_server", 
                        name="behavior_server", 
                        output="screen", 
                        parameters=common_params, 
                        remappings=cmd_vel_remapping
                    ),
                    Node(
                        package="nav2_bt_navigator", 
                        executable="bt_navigator", 
                        name="bt_navigator", 
                        output="screen", 
                        parameters=common_params, 
                        remappings=NAV_REMAPPINGS
                    )
                ])

        case Navigation.TELEOP:
            pass
        case _:
            pass

    if navigation == Navigation.NAV2 or localization == Localization.AMCL:
        for robot in robots:
            robot_name = robot["name"]
            if per_robot_lc_nodes[robot_name]:
                per_robot_nodes[robot_name].append(
                    Node(
                        package="nav2_lifecycle_manager", 
                        executable="lifecycle_manager", 
                        name="lifecycle_manager_navigation", 
                        output="screen",
                        parameters=[
                            {"use_sim_time": use_sim_time}, 
                            {"autostart": True}, 
                            {"node_names": per_robot_lc_nodes[robot_name]}
                        ]
                    )
                )
    # ==========================================================================
    # 2. CONNECTED EXPLORERS
    # ==========================================================================
    robots_name_list = [r["name"] for r in robots]
    is_3d = (dimension == Dimensions.DIMENSION_3)

    global_nodes.append(
        Node(
            package="connected_explorers_global_supervisor",
            executable="supervisor_node",
            parameters=[
                {"number_of_robots": len(robots)}, #TODO melhorar isso aqui
                {"robot_name_prefix": "robot"}, #TODO remover hardcoded
                {"is_3d_mode": is_3d}
            ],
        )
    )
            
            
    global_nodes.append(
        Node(
            package="connected_explorers_connections",
            executable="distance_watcher_node",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"number_of_robots":len(robots)}, #TODO melhorar isso aqui
                {"robot_name_prefix":"robot"}, #TODO remover hardcoded
                {"is_3d_mode":is_3d},
                {"los_alpha":los_alpha},
                {"los_beta":los_beta},
                {"distance_alpha":distance_alpha},
                {"distance_beta":distance_beta},
            ],
        )
    )
    
    global_nodes.append(
        Node(
            package="line_viewer",
            executable="RobotsPositionNode",
            name="RobotsPositionNode",
            parameters=[
                {"robots_list": robots_name_list}, #TODO tem nó pedindo robots list e nó pedindo number of robots
                {"reference_frame": "map"}, #TODO remover hardcoded
            ],
        )
    )
    
    global_nodes.append(
        Node(
            package="illustrator",
            executable="illustrator_node",
            name="illustrator_node",
            parameters=[
                {"number_of_robots": len(robots)}, #TODO melhorar isso aqui
                {"robot_name_prefix": "robot"}, #TODO remover hardcoded
                {"reference_frame": "map"}, #TODO remover hardcoded
            ],
        )
    )

            
    global_nodes.append(
        Node(
            package="connected_explorers_messagery",
            executable="router_node",
            parameters=[
                {"number_of_robots":len(robots)} #TODO melhorar isso aqui
            ]
        )
    )
    
    for i,robot in enumerate(robots):
        robot_name = robot["name"]
        per_robot_nodes[robot_name].extend([
        Node(
            package="connected_explorers_messagery",
            executable="robot_inbox_node",
            parameters=[
                {"robot_id":i+1},
            ]
        ),

        Node(
            package="connected_explorers_messagery",
            executable="robot_outbox_node",
            parameters=[
                {"robot_id":i+1},
            ]
        ),

        Node(
            package="connected_explorers_laplacian_matrix",
            executable="laplacian_matrix_estimator_node",
            parameters=[
                {"number_of_robots":len(robots)},
                {"robot_index":i+1},
                {"is_3d_mode":dimension == Dimensions.DIMENSION_3},
                {"los_alpha":los_alpha},
                {"los_beta":los_beta},
                {"distance_alpha":distance_alpha},
                {"distance_beta":distance_beta}
            ]
        ),

        Node(
            package="line_viewer",
            executable="SingleRobotControllerNode",
            name=f"RobotControllerNode_{robot_name}",
            parameters=[
                {"robots_list": [robot["name"] for robot in robots]},
                {"robot_number": i + 1},
                {"robot_role": robot["function"]},
                {"holonomic": kinematics == Kinematics.HOLONOMIC},
                {"is_3d_mode": dimension == Dimensions.DIMENSION_3},
                {"epsilon": 0.3},
                {"gamma": 3.0},
                {"control_period": 0.05},
                {"max_vel": 0.5},
                {"real_max_w": 1.5},
                {"l_point": 0.2},
                {"lambda_conn_threshold": 0.9},
                {"conn_desired_mag": 0.95},
                {"slack_weight": 1e8},
                {"collision_detection_dist": 1.5},
                {"collision_safe_dist": 1.0}
            ],
        )
    ])








    final_launch_list = []
    final_launch_list.extend(global_nodes)
    
    for robot_name, nodes in per_robot_nodes.items():
        robot_group = GroupAction([
            PushRosNamespace(robot_name),
            *nodes
        ])
        final_launch_list.append(robot_group)

    return LaunchDescription(final_launch_list)