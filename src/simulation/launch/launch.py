import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    simulation_dir = get_package_share_directory('simulation')
    config_dir = os.path.join(get_package_share_directory('simulation'), 'config')
    config_file = os.path.join(config_dir, 'manipulator.yaml')

    
    # 로봇 SDF 경로
    robot_sdf = os.path.join(simulation_dir, 'urdf', 'model.sdf')
    
    # 월드 파일 경로
    world_file = os.path.join(simulation_dir, 'maps', 'mainmap.world')
    
    # Gazebo 서버 실행
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py'
        )]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo 클라이언트 실행
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'
        )])
    )
    
    # 로봇 스폰 노드
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', robot_sdf,
            '-x', '5', '-y', '3', '-z', '0.5',
            '-R', '0', '-P', '0', '-Y', '1.57'
        ],
        output='screen',
        # 다음 줄을 추가하여 출력을 파일로 리디렉션
        on_exit=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py'
            )),
            launch_arguments={'world': world_file}.items()
        )
    )
    
    # gnome-terminal을 사용하여 teleop_twist_keyboard 노드 실행
    teleop_twist_keyboard = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard', '--ros-args', '--remap', '/cmd_vel:=/cmd_vel'],
        output='screen'
    )
    
        # 컨트롤러 매니저 노드
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        spawn_entity,
        teleop_twist_keyboard,
        controller_manager,
    ])
