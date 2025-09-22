import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Definir modelo do TurtleBot3
    turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')
    
    # Caminhos dos pacotes
    pkg_my_worlds = get_package_share_directory('my_worlds')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Caminho do mundo personalizado
    world_path = os.path.join(pkg_my_worlds, 'worlds', 'new_bug_world.sdf')
    
    # 1. Iniciar Gazebo com mundo personalizado
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'use_sim_time': 'true'
        }.items()
    )
    
    # 2. Robot State Publisher do TurtleBot3
    robot_state_publisher = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )
    
    # 3. Spawn do robô
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-topic', 'robot_description',
                    '-name', 'turtlebot3_waffle',
                    '-x', '0.0',
                    '-y', '0.0', 
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )
    
    # 4. Ponte ROS-Gazebo manual (sem depender do launch file)
    ros_gz_bridge = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    # Clock para sincronização
                    'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    # Estados das juntas
                    'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    # Odometria
                    'odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    # TF
                    'tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    # Comando de velocidade (TwistStamped para TurtleBot3)
                    'cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
                    # IMU
                    'imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    # LaserScan
                    'scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    # Camera info (se necessário)
                    'camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '--ros-args',
                    '-p', 'use_sim_time:=true'
                ],
                output='screen',
                parameters=[{
                    'use_sim_time': True
                }]
            )
        ]
    )
    
    # 5. Verificação automática dos sensores
    sensor_check = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', '''
                    echo "Verificando sensores após ponte..."
                    sleep 3
                    
                    echo "Tópicos ROS2:"
                    ros2 topic list | grep -E "(scan|odom|cmd_vel)" || echo "Tópicos não encontrados ainda"
                    
                    echo -e "\nTestando /scan..."
                    timeout 5s ros2 topic echo /scan --once > /dev/null 2>&1 && echo "LaserScan funcionando" || echo "LaserScan aguardando..."
                    
                    echo "Testando /odom..."
                    timeout 5s ros2 topic echo /odom --once > /dev/null 2>&1 && echo "Odometria funcionando" || echo "Odometria aguardando..."
                    
                    echo -e "\nExecute: ros2 run bug_algorithms_pkg bug1_node"
                '''],
                output='screen'
            )
        ]
    )
    
    # 6. Bug1 Node
    bug1_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='bug_algorithms_pkg',
                executable='bug1_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        turtlebot3_model,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        sensor_check,
        bug1_node,
    ])