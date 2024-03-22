import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('uned_limo_gazebo')
    map_file = os.path.join(pkg_dir, 'limo_nav2', 'config', 'maps', 'circuito_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_path = os.path.join(pkg_dir, 'limo_description', 'rviz', 'display_model.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Ruta al mapa a cargar'),
             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Usa el reloj de Ignition (if true)'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
            launch_arguments=[
                ('map', map_file),
                ('use_sim_time', use_sim_time),
            ],
        ),

        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
        ),
    ])