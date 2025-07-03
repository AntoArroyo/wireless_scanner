from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wireless_scanner',
            executable='wireless_scanner',
            name='wireless_scanner',
            output='screen',
            #prefix='sudo -E'
            # Launch the node with root access (GPIO) in a shell
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        )
    ])