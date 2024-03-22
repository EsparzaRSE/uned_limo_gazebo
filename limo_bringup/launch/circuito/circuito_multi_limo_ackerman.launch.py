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
    xacro_path = os.path.join(get_package_share_path('uned_limo_gazebo'), 'limo_description', 'urdf', 'limo_ackerman.xacro')
    world_path = os.path.join(pkg_dir, 'limo_bringup', 'worlds', 'circuito.sdf')
    rviz_config_path = os.path.join(pkg_dir, 'limo_description', 'rviz', 'multi_model_display_ackerman.rviz')
    bridge_params = [
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'ackerman', 'limo0_ackerman.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'ackerman', 'limo1_ackerman.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'ackerman', 'limo2_ackerman.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'ackerman', 'limo3_ackerman.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'ackerman', 'limo4_ackerman.yaml')
    ]   

    # Añadir o quitar robots al array para spawnearlos. El bridge en YAML está preparado para máx 5 robots
    # Añadir la barra al final de los nombres #
    robot_names = ['limo0_ackerman/', 'limo1_ackerman/', 'limo2_ackerman/', 'limo3_ackerman/', 'limo4_ackerman/']
    
    urdf_files = []
    robot_descriptions = []

    for robot_name in robot_names:

        # Crea un urdf temporal para spawnear el robot en el mundo
        urdf = subprocess.check_output(['xacro', xacro_path, f"robot_name:={robot_name}"])
    
        urdf_file = tempfile.NamedTemporaryFile(delete=False)
        urdf_file.write(urdf)
        urdf_file.close()

        # Guarda la ruta del archivo en la lista urdf_files
        urdf_files.append(urdf_file.name)

        # Crea la descripción del robot para usarla en el nodo robot_state_publisher
        # Los espacios en los parámetros muy importante ponerlos o da error
        robot_description = ParameterValue(Command(['xacro ', xacro_path, ' robot_name:=' + robot_name]), 
                                                   value_type=str) 

        robot_descriptions.append(robot_description)

    # Configuración
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    robot_poses = [
        {"x": "0", "y": "0.1739", "z": "0.0522"},
        {"x": "0", "y": "0.4650", "z": "0.0522"},
        {"x": "-0.5598", "y": "0.1739", "z": "0.0522"},
        {"x": "-0.5598", "y": "0.4650", "z": "0.0522"},
        {"x": "-1.1496", "y": "0.1739", "z": "0.0522"},
    ]

    # Lanzamiento de los nodos
    ld = LaunchDescription()

    # Lanza Ignition Gazebo con el mundo
    ld.add_action(ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    ))

    for i in range(len(urdf_files)):

        # Carga el robot en el mundo
        ld.add_action(Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=["-file", urdf_files[i],
                   "-name", robot_names[i][:-1],
                   "-x", robot_poses[i]["x"],
                   "-y", robot_poses[i]["y"],
                   "-z", robot_poses[i]["z"]
                  ]
        ))

    # Bridges entre ROS2 e Ignition
        ld.add_action(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_names[i][:-1],
            arguments=['--ros-args',
                   '-p',
                   f'config_file:={bridge_params[i]}',
                   ],
            output='screen',
        ))

        ld.add_action(Node(
            package='ros_gz_image',
            executable='image_bridge',
            namespace=robot_names[i][:-1],
            arguments=['color/image_raw'],
            output='screen',
        ))

        # Robot state publisher
        ld.add_action(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_names[i][:-1],
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_descriptions[i]}],
            output="screen"
        ))

    return ld