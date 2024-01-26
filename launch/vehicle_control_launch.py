
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()  

    #autonomous driving controller
    gap_follower_node = Node(
        package='gap_follow',
        executable='gap_follower',
        output='screen',
        name='gap_follower',
        parameters=[],
        remappings=[('/drive', '/to_drive')] #gap following publishes to vehicle_control_node
    )

    #vehicle control - manual intervention for autonomous driving
    joy_config = os.path.join(
        get_package_share_directory('vehicle_control'),
        'config',
        'gamepad.yaml'
        )
    joy_config_dict = yaml.safe_load(open(joy_config, 'r'))
    vehicle_control_node = Node(
        package='vehicle_control',
        executable='vehicle_control',
        output='screen',
        name='vehicle_control',
        parameters=[joy_config_dict]
    )

    #driver for the gamepad
    joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        emulate_tty="true"
    )

    #driver for lidar
    lidar_launchfile = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare("rplidar_ros"), '/launch', '/rplidar_s3_launch.py'])
                        )

    #driver for vesc
    vesc_launchfile = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare("vesc_driver"), '/launch', '/vesc_driver_node.py'])
                        )


    # finalize
    ld.add_action(vehicle_control_node)
    ld.add_action(gap_follower_node)
    ld.add_action(joy_linux_node)
    ld.add_action(lidar_launchfile)
    ld.add_action(vesc_launchfile)

    return ld