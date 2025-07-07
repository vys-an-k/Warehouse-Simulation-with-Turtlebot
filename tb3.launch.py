import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
from launch.actions import LogInfo
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 1) world file arg
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("warehouse_sim"),
            "worlds", "warehouse.world"
        ),
        description="Path to Gazebo world file"
    )

    # 2) bring up Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items()
    )

    # 3) common paths
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_description"),
        "urdf",
        "turtlebot3_burger.urdf"
    )
    slam_yaml = os.path.join(
        get_package_share_directory("warehouse_sim"),
        "maps",
        "slam_params.yaml"
    )

    log_yaml = LogInfo(msg=["Loading SLAM params from: ", slam_yaml])

    map_yaml = os.path.expanduser("~/warehouse_map.yaml")
    nav2_params = os.path.join(get_package_share_directory('warehouse_sim'),
        'maps', 'nav2_params.yaml')
    
    tm = Node(
            package='warehouse_sim',
            executable='task_manager',
            name='task_manager',
            output='screen',
            parameters=[{
                'tasks_file': os.path.join(
                    get_package_share_directory('warehouse_sim'),
                    'tasks', 'tasks.yaml'
                )
            }]
        )
    robot_groups = []
    s = [1.0, 1.0]
    t = [0.0, 4.0]
    for i in range(1):
        ns = f"robot_{i}"
        group = GroupAction([
            # this pushes /robot_i for EVERYTHING inside here
            PushRosNamespace(ns),
        # robot_state_publisher under /robot_i
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{
                    "robot_description": open(urdf_path).read(),
                    "use_sim_time": True
                }],
                output="screen"

),

          

            # spawn entity 
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-database', 'turtlebot3_burger',
                '-entity', ns,
                '-robot_namespace', ns,
                '-x', str(s[i]),
                '-y', str(t[i]),
                '-z', '0.01'
            ]
        ),

            # your own executor node
            Node(
                package="warehouse_sim",
                executable="robot_executor",
                name = "robot_executor",
                output="screen",
                parameters=[{"robot_id": i, "use_sim_time": True}],
            )
    ])
        

        robot_groups.append(group)
        map_odom_broadcaster = Node(
        package='warehouse_sim',
        executable='map_odom_broadcaster',   # or use static_transform_publisher w/ --timestamp
        name='map_odom_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    return LaunchDescription([
        world_arg,
        gazebo,
        log_yaml,
        tm,
        map_odom_broadcaster,
        *robot_groups
    ])
