import os
import subprocess
import tempfile
from ament_index_python import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Rutas
    pkg_dir = get_package_share_directory('uned_limo_gazebo')
    xacro_path = os.path.join(get_package_share_path('uned_limo_gazebo'), 'limo_description', 'urdf', 'limo_four_diff.xacro')
    world_path = os.path.join(pkg_dir, 'limo_bringup', 'worlds', 'model_laboratorio.sdf')
    rviz_config_path = os.path.join(pkg_dir, 'limo_description', 'rviz', 'model_display.rviz')
    bridge_params = os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'one_limo.yaml')
 
    # Configuración y parámetros
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), 
                                       value_type=str)
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Lanzamiento de los nodos

    ld = LaunchDescription([

        # Lanza Ignition Gazebo con el mundo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_path],
            output='screen'
        ),

        # Bridges entre ROS2 e Ignition
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args',
                        '-p',
                        f'config_file:={bridge_params}',
                        ],
            output='screen',
        ),
        
         Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['color/image_raw'],
            output='screen',
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
            output="screen"
             
        ),
    ])

    return ld