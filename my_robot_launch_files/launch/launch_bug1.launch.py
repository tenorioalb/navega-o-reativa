import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_worlds = get_package_share_directory('my_worlds')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_bug_algorithms_pkg = get_package_share_directory('bug_algorithms_pkg')

    # Define o caminho de recursos do Gazebo para que ele encontre os modelos
    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot3_gazebo, 'worlds'),
            os.path.join(pkg_my_worlds, 'worlds'),
            os.path.join(pkg_turtlebot3_description)
        ]
    )

    # Caminho para o seu mundo personalizado
    world_path = os.path.join(pkg_my_worlds, 'worlds', 'new_bug_world.sdf')

    # Lançar o simulador Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_path}.items()
    )

    # Publicar o estado do robô a partir do arquivo URDF
    urdf_file_name = 'turtlebot3_waffle.urdf'
    urdf_path = os.path.join(pkg_turtlebot3_description, 'urdf', urdf_file_name)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read(), 'use_sim_time': True}]
    )

    # Spawnar o robô no mundo do Gazebo
    spawn_turtlebot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'turtlebot3_waffle',
                   '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )

    # Lançar o nó do algoritmo Bug 1
    bug1_node = Node(
        package='bug_algorithms_pkg',
        executable='bug1_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_resource_path,
        gazebo_launch,
        robot_state_publisher,
        spawn_turtlebot_node,
        bug1_node
    ])
