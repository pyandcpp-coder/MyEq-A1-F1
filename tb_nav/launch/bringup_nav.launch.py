import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Paths
    tb_sim_pkg = FindPackageShare('tb_sim').find('tb_sim')
    tb_nav_pkg = FindPackageShare('tb_nav').find('tb_nav')

    # --- Launch Arguments ---
    map_yaml_file = os.path.join(tb_nav_pkg, 'maps', 'my_world_map.yaml')
    nav2_params_file = os.path.join(tb_nav_pkg, 'config', 'nav2_params.yaml')

    # --- Simulation ---
    start_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb_sim_pkg, 'launch', 'start_world_and_robot.launch.py')
        )
    )

    # --- Map Server ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_yaml_file}]
    )

    # --- AMCL Localization ---
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    # --- Lifecycle Manager for Localization ---
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # --- Nav2 Navigation Stack (controller, planner, behavior, bt_navigator, etc.) ---
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb_nav_pkg, 'launch', 'nav2_launch_include_file.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    # --- Static Transform Publisher (map -> odom) ---
    # This is temporary until AMCL takes over after you set initial pose in RViz
    static_map_to_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # --- Launch Description ---
    ld = LaunchDescription()

    # Add nodes
    ld.add_action(start_simulation_cmd)
    ld.add_action(static_map_to_odom_publisher)  # Add this BEFORE map_server
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_localization)
    ld.add_action(start_navigation_cmd)

    return ld