from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    launch_d = LaunchDescription()
    
    liconic_client = Node(
            package = 'liconic_client',
            namespace = 'std_ns',
            executable = 'liconic_client',
            output = "screen",
            name='liconicNode'
        )
    
    launch_d = LaunchDescription()

    launch_d.add_action(liconic_client)

    return launch_d