import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

def generate_launch_description():
    tb_sim_dir = get_package_share_directory('tb_sim')
    world_file = os.path.join(tb_sim_dir, 'worlds', 'final.sdf')
    rviz_config_file = os.path.join(tb_sim_dir, 'rviz', 'myeq_rviz_config.rviz')
    
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    urdf_file = os.path.join(turtlebot3_gazebo_dir, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', TURTLEBOT3_MODEL, '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.01'],
        output='screen'
    )
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_turtlebot,
        rviz_node
    ])

