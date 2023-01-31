from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    launch_d = LaunchDescription()
    
    lyconic_client = Node(
            package = 'lyconic_client',
            namespace = 'std_ns',
            executable = 'lyconic_client',
            output = "screen",
            name='lyconicNode'
        ),
    ])
