from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flir_lepton_human_detector',
            executable='threshold_contour_detector_node',
            name='threshold_detector',
            output='screen'
        )
    ])
