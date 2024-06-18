import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    simulation_dir = get_package_share_directory('simulation')
    
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
            '-x', '1', '-y', '2', '-z', '0.5',
            '-R', '0', '-P', '0', '-Y', '1.57'
        ],
        output='screen'
    )
    

    # 로봇 컨트롤러 노드 추가
    robot_controller = Node(
        package='robot_controller',  
        executable='robot_controller',  
        name='robot_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        spawn_entity,
        robot_controller,
    ])
