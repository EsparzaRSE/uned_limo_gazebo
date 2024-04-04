import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    # Rutas
    pkg_dir = get_package_share_directory('uned_limo_gazebo')
    xacro_path = os.path.join(pkg_dir, 'limo_description', 'urdf', 'limo_four_diff.xacro')
    urdf_path = os.path.join(pkg_dir, 'limo_description', 'urdf', 'limo')
    world_path = os.path.join(pkg_dir, 'limo_bringup', 'worlds', 'circuito.sdf')
    rviz_config_path = os.path.join(pkg_dir, 'limo_description', 'rviz', 'model_display.rviz')
    one_bridge_params =  os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'one_limo.yaml')
    bridge_params = [
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'four_diff', 'limo0_four_diff.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'four_diff', 'limo1_four_diff.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'four_diff', 'limo2_four_diff.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'four_diff', 'limo3_four_diff.yaml'),
                    os.path.join(pkg_dir, 'limo_bringup', 'param', 'bridge', 'four_diff', 'limo4_four_diff.yaml')
    ]

    # Añadir o quitar robots al array para spawnearlos. El bridge en YAML está preparado para máx 5 robots
    # Añadir la barra al final de los nombres 
    # Para un robot usar mejor el namespace '' 
    # 'limo0_four_diff/', 'limo1_four_diff/', 'limo2_four_diff/', 'limo3_four_diff/', 'limo4_four_diff/'
    robot_names = ['']

    # Configuración
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    robot_poses = [
        {"x": "0", "y": "0.1739", "z": "0.0522"},
        {"x": "0", "y": "0.4650", "z": "0.0522"},
        {"x": "-0.5598", "y": "0.1739", "z": "0.0522"},
        {"x": "-0.5598", "y": "0.4650", "z": "0.0522"},
        {"x": "-1.1496", "y": "0.1739", "z": "0.0522"},
    ]

    # Crea una descripción de lanzamiento
    ld = LaunchDescription()

    # Lanza Ignition Gazebo con el mundo
    ld.add_action(ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    ))

    for i in range(len(robot_names)):

        # Genera una ruta de archivo única para cada robot
        urdf_path_limo = urdf_path_limo = f"{urdf_path}{'' if robot_names[i] == '' else i}_four_diff.urdf"

        # Convierte xacro a urdf y lo guarda en los shared
        ld.add_action(ExecuteProcess(
                cmd=['xacro', xacro_path, f"robot_namespace:={robot_names[i]}", '-o', urdf_path_limo],
                output='screen'
            ))

        # Carga el robot en el mundo (no funciona directamente desde el xacro, da error)
        ld.add_action(Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=["-file", urdf_path_limo,
                        "-name", robot_names[i][:-1] if robot_names[i] != '' else 'limo_four_diff',
                        "-x", robot_poses[i]["x"],
                        "-y", robot_poses[i]["y"],
                        "-z", robot_poses[i]["z"]
                ]
        ))

        # Bridges entre ROS2 e Ignition
        ld.add_action(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['--ros-args',
                        '-p',
                        f'config_file:={bridge_params[i] if robot_names[i] != "" else one_bridge_params}',
                ],
                output='screen',
        ))

        ld.add_action(Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=[robot_names[i] + 'color/image_raw'],
                output='screen',
        ))

        # Robot state publisher
        ld.add_action(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_names[i],
                parameters=[{'robot_description': Command(['xacro ', xacro_path, ' robot_namespace:=' + robot_names[i]]),
                                'use_sim_time': use_sim_time}
                ],        
        ))

        # Joint state publisher
        ld.add_action(Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=robot_names[i],
                parameters=[{'use_sim_time' : use_sim_time}],
                output="screen"
        ))  

    return ld