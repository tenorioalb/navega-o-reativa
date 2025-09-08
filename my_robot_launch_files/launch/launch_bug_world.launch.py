import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_my_robot_launch_files = get_package_share_directory('my_robot_launch_files')

    # Caminho para o seu mundo personalizado
    world_path = os.path.join(pkg_my_robot_launch_files, 'worlds', 'simple_bug_world.sdf')

    # Declara o argumento 'world' que será passado para o Gazebo
    world = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Path to the Gazebo world file'
    )

    # Inicia o Gazebo com o seu mundo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

# Re-use your spawn_turtlebot_launch section
    spawn_turtlebot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            )
        )
    )

    return LaunchDescription([
        world,
        gazebo_launch,
        spawn_turtlebot_launch
    ])