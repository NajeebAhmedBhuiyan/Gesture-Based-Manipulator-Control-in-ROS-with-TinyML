from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MoveIt Service Node
        Node(
            package='kinova_control',
            executable='moveit_service',
            name='moveit_service',
            output='screen'
        ),
        
        # Pygame Client Node (optional)
        Node(
            package='kinova_control',
            executable='client_kinova_control.py',
            name='moveit_client',
            output='screen'
        ),
        
        # Gesture Controller Node
        Node(
            package='kinova_control',
            executable='gesture_controller.py',
            name='gesture_controller',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0'  # Can parameterize this
            }]
        )
    ])