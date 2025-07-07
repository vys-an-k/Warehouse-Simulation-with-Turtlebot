import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ############################################################################
    # 1) TWO SEPARATE NAMESPACES FOR EACH ROBOT
    ############################################################################
    robot1_ns = 'robot_0'
    robot2_ns = 'robot_1'
    ############################################################################
    # 2) POINT TO THE BT XML (UNCHANGED FROM NAV2 PACKAGES)
    ############################################################################
    bt_xml = os.path.expanduser('~/bt_xml.xml')
    nav2_params = os.path.join(
        FindPackageShare('warehouse_sim').find('warehouse_sim'),
        'maps',
        'nav2_params.yaml'
    )

    ############################################################################
    # 3) “COMMON NODES”: ALL THE NAV2 COMPOSABLES
    #    (map_server, amcl, planner_server, controller_server, bt_navigator, spin)
    #    None of these specify namespace—they inherit from the container’s namespace.
    ############################################################################
    def common_nodes(ns):
        scan_in  = f'/{ns}/scan'
        scan_out = f'/{ns}/scan_filtered'
        return [
            
            # ─── Map Server ────────────────────────────────────────────
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                namespace = ns,
                parameters=[nav2_params, {
            'use_sim_time': True,
            'yaml_filename': os.path.expanduser('~/warehouse_map.yaml'),
          }]
            ),
            # ─── AMCL ──────────────────────────────────────────────────
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                namespace = ns,
                remappings=[('scan', scan_in)],
                parameters=[nav2_params]
        ),
            # ─── Planner Server ───────────────────────────────────────
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                namespace = ns,
                parameters=[nav2_params],
                remappings=[('scan', scan_in)],
),
                  
        
            # ─── Controller Server ────────────────────────────────────
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                namespace = ns,
                remappings=[('scan', scan_in)],
                parameters=[nav2_params, {
        'use_sim_time': True,
        'controller_plugins': ['FollowPath'],
        'FollowPath': {
            'FollowPath.plugin': 'dwb_core::DWBLocalPlanner',
                    'FollowPath.critics': [
                        'GoalAlignCritic',
                        'PathDistCritic',
                        'ObstacleFootprintCritic',
                        'GoalDistCritic'
                    ],
                    'GoalAlignCritic.scale': 24.0,
                    'PathDistCritic.scale': 32.0,
                    'ObstacleFootprintCritic.scale': 0.01,
                    'GoalDistCritic.scale': 20.0,

            'use_collision_detection': True,
            'lookahead_dist': 0.75,
            'min_lookahead_dist': 0.4,
            'max_lookahead_dist': 1.5,
            'use_rotate_to_heading': True,
            'rotate_to_heading_min_angle': 0.785,
            'rotate_to_heading_angular_vel': 0.6,
        },
        'goal_checker_plugins': ['goal_checker'],
        'goal_checker': {
            'plugin': 'nav2_controller::SimpleGoalChecker',
            'xy_goal_tolerance': 0.25,
            'yaw_goal_tolerance': 0.25,
        },
        'progress_checker_plugin': 'progress_checker',
        'progress_checker': {
            'plugin': 'nav2_controller::SimpleProgressChecker',
            'required_movement_radius': 0.5,
            'movement_time_allowance': 10.0,
        },
        'local_costmap/obstacle_layer/observation_sources': 'scan',
        'local_costmap/obstacle_layer/scan/topic': scan_in,
        'local_costmap/obstacle_layer/scan/data_type': 'LaserScan',
        'local_costmap/obstacle_layer/scan/min_obstacle_range': 0.2,
        'local_costmap/obstacle_layer/scan/max_obstacle_range': 3.0,
        'global_costmap/obstacle_layer/observation_sources': 'scan',
        'global_costmap/obstacle_layer/scan/topic': scan_in,
        'global_costmap/obstacle_layer/scan/data_type': 'LaserScan',
        'global_costmap/obstacle_layer/scan/min_obstacle_range': 0.2,
        'global_costmap/obstacle_layer/scan/max_obstacle_range': 3.0,
        
            }]
        ),        

            # ─── BT Navigator ─────────────────────────────────────────
            ComposableNode(
    package='nav2_bt_navigator',
    plugin='nav2_bt_navigator::BtNavigator',
    name='bt_navigator',
    namespace=ns,
    parameters=[nav2_params, {
            'use_sim_time': True,
            'bt_xml_filename': bt_xml,

            'server_names': {
              'compute_path_to_pose':       f'/{ns}/navigate_to_pose',
              'compute_path_through_poses': f'/{ns}/navigate_through_poses',
              'follow_path':                f'/{ns}/follow_path'
            },
            'global_costmap_service': f'/{ns}/global_costmap/clear_entirely_global_costmap',
            'local_costmap_service':  f'/{ns}/local_costmap/clear_entirely_local_costmap'
          },],
    remappings=[
                    ('compute_path_through_poses', f'/{ns}/navigate_through_poses'),
                    ('compute_path_to_pose',      f'/{ns}/navigate_to_pose'),
                    ('follow_path',               f'/{ns}/follow_path'),
                    ('scan', scan_in)
                ]
)
            
        ]

    ############################################################################
    # 4) LIFECYCLE MANAGERS (3 STAGES), ALL UNDER namespace=ns
    #
    #    We return a list containing:
    #      • Node(...) for localization manager  @ t = 0 relative
    #      • TimerAction(4.0, Node(... nav manager))  @ t = 4.0 s relative
    #      • TimerAction(8.0, Node(... bt manager))   @ t = 8.0 s relative
    #
    #    *When we hook this list into OnProcessStart(... on_start=[ ... ]), 
    #     it effectively means:*
    #       - At (launch + 2 s) → run lifecycle_nodes(robot_X)[0]
    #       - At (launch + 2 s + 4 s = 6 s) → run lifecycle_nodes(robot_X)[1]
    #       - At (launch + 2 s + 8 s = 10 s) → run lifecycle_nodes(robot_X)[2]
    #
    def lifecycle_nodes(ns):
        return [
            # ─── 1) Activate map_server + amcl ───────────────────────────────────
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                namespace=ns,      # <── This lives at /robot_X
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']
                }]
            ),
            # ─── 2) After 4 s, activate planner_server + controller_server ─────────
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation',
                        namespace=ns,  # <── This lives at /robot_X
                        output='screen',
                        parameters=[{
                            'use_sim_time': True,
                            'autostart': True,
                            'node_names': ['planner_server', 'controller_server', 'behavior_server']
                        }]
                    )
                ]
            ),
            # ─── 3) After 8 s, activate BT navigator ───────────────────────────────
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_bt',
                        namespace=ns,  # <── This lives at /robot_X
                        output='screen',
                        parameters=[{
                            'use_sim_time': True,
                            'autostart': True,
                            'node_names': ['bt_navigator']
                        }]
                    )
                ]
            )
        ]

    ############################################################################
    # 5) ROBOT_0: NAV2 CONTAINER + LIFECYCLE MANAGERS + STATIC TF
    ############################################################################
    container1 = ComposableNodeContainer(
        name='nav2_container_0',
        namespace=robot1_ns,       # <── EVERYTHING inside runs at /robot_0/...
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=common_nodes(robot1_ns),
        output='screen',
        emulate_tty=True
    )

    behavior_r0 = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=robot1_ns,
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # 2 s after container1 “starts,” kick off all three lifecycle managers (via lifecycle_nodes)
    start_lifecycle1 = RegisterEventHandler(
        OnProcessStart(
            target_action=container1,
            on_start=[TimerAction(
                period=4.0,
                actions=lifecycle_nodes(robot1_ns)
            )]
        )
    )

    # We still need a static map→odom TF under /robot_0
    tf_r0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        namespace=robot1_ns,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    initial_pose_r0 = TimerAction(
        period=6.0,  # ← wait until AMCL is ACTIVE
        actions=[Node(
            package='warehouse_sim',
            executable='initial_pose_publisher',
            name='initial_pose_pub_robot_0',
            namespace=robot1_ns,
            output='screen',
            parameters=[{
                'robot_ns': 'robot_0',
                'x':      1.0,
                'y':      0.0,
                'yaw':    0.0
            }]
        )]
    )

    delayed_initial_pose_r0 = TimerAction(
        period=6.0,  # ↖– wait 6 seconds after launch
        actions=[Node(
            package='warehouse_sim',
            executable='initial_pose_publisher',
            name='initial_pose_pub_robot_0',
            namespace=robot1_ns,
            output='screen',
            parameters=[{
                'robot_ns': 'robot_0',
                'x': 2.0,
                'y': 1.0,
                'yaw': 0.0
            }]
        )]
    )

    ############################################################################
    # 6) ROBOT_1: NAV2 CONTAINER + LIFECYCLE MANAGERS + STATIC TF
    ############################################################################
    container2 = ComposableNodeContainer(
        name='nav2_container_1',
        namespace=robot2_ns,       # <── EVERYTHING inside runs at /robot_1/...
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=common_nodes(robot2_ns),
        output='screen',
        emulate_tty=True
)

    behavior_r1 = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=robot2_ns,
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 2 s after container2 “starts,” kick off its three lifecycle managers
    start_lifecycle2 = RegisterEventHandler(
        OnProcessStart(
            target_action=container2,
            on_start=[TimerAction(
                period=4.0,
                actions=lifecycle_nodes(robot2_ns)
            )]
        )
    )

    # Static map→odom TF under /robot_1
    tf_r1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        namespace=robot2_ns,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    initial_pose_r1 = TimerAction(
        period=6.0,  # ← wait until AMCL is ACTIVE
        actions=[Node(
            package='warehouse_sim',
            executable='initial_pose_publisher',
            name='initial_pose_pub_robot_1',
            namespace=robot2_ns,
            output='screen',
            parameters=[{
                'robot_ns': 'robot_1',
                'x':      1.0,
                'y':      3.0,
                'yaw':    0.0
            }]
        )]
    )

    spin_r0 = Node(
    package='nav2_behaviors',
    executable='spin',
    name='spin',
    namespace=robot1_ns,
    output='screen',
    parameters=[{'use_sim_time': True}]
)

    spin_r1 = Node(
    package='nav2_behaviors',
    executable='spin',
    name='spin',
    namespace=robot2_ns,
    output='screen',
    parameters=[{'use_sim_time': True}]
)
    
    map_odom_broadcaster = Node(
        package='warehouse_sim',
        executable='map_odom_broadcaster',
        name='map_odom_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ########################################################
    # 7) FINAL LAUNCH DESCRIPTION
    ############################################################################
    return LaunchDescription([
        # ─── robot_0 NAV2 ─────────────────────────────────────────────────────
        map_odom_broadcaster,
        container1,
        behavior_r0,
        start_lifecycle1,
        initial_pose_r0,
        # ─── robot_1 NAV2 ─────────────────────────────────────────────────────
        container2,
        behavior_r1,
        start_lifecycle2,
        initial_pose_r1,
    ])