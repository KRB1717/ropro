from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        teleop_twist_keyboard,
    ])

