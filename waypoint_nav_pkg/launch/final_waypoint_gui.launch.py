import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    tb_nav_pkg_dir = get_package_share_directory('tb_nav')
    nav_launch_file = os.path.join(tb_nav_pkg_dir, 'launch', 'bringup_nav.launch.py')

    start_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file)
    )

    start_waypoint_manager_cmd = Node(
        package='waypoint_nav_pkg',
        executable='waypoint_manager.py',
        name='waypoint_manager_node',
        output='screen'
    )

    start_waypoint_gui_cmd = Node(
        package='waypoint_nav_pkg',
        executable='waypoint_gui.py',
        name='waypoint_gui_node',
        output='screen',
        emulate_tty=True  
    )

    return LaunchDescription([
        start_nav_cmd,
        start_waypoint_manager_cmd,
        start_waypoint_gui_cmd
    ])