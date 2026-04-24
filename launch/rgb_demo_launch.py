
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    ld = LaunchDescription()  

    rgb_demo_node = Node(
        package='vehicle_control',
        executable='rgb_demo',
        output='screen',
        name='rgb_demo'
    )

    aestaetic_node = Node(
        package='aesthetic_control',
        executable='aesthetic_control',
        output='screen',
        name='aesthetic_control',
        parameters=[]
    )

    # finalize
    ld.add_action(rgb_demo_node)
    ld.add_action(aestaetic_node)

    return ld
