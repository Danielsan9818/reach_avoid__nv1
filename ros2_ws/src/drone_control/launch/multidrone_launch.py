from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Pursuer 1
        Node(
            package='drone_control',
            executable='controller',
            namespace='Pursuer_1',
            name='controller',
            parameters=[{'drone_id': 1}]
        ),
        # Pursuer 2
        Node(
            package='drone_control',
            executable='controller',
            namespace='Pursuer_2',
            name='controller',
            parameters=[{'drone_id': 2}]
        ),
        # Pursuer 3
        Node(
            package='drone_control',
            executable='controller',
            namespace='Pursuer_3',
            name='controller',
            parameters=[{'drone_id': 3}]
        ),
        # Pursuer 4
        Node(
            package='drone_control',
            executable='controller',
            namespace='Pursuer_4',
            name='controller',
            parameters=[{'drone_id': 4}]
        ),
        # Evader
        Node(
            package='drone_control',
            executable='controller',
            namespace='evader',
            name='controller',
            parameters=[{'drone_id': 5}]
        )
        
    ])