
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
        name="joy_linux_node",
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
                            FindPackageShare("vesc_driver"), '/launch', '/vesc_driver_node.launch.py'])
                        )

    vesc_odometer = IncludeLaunchDescription(
                        AnyLaunchDescriptionSource([
                            FindPackageShare("vesc_ackermann"), '/launch', '/vesc_to_odom_node.launch.xml'])
                        )

    #ackermann driver for vesc
    #but instead of listening to /ackermann_cmd, we want to listen to /drive for input messages
    vesc_ackermann = Node(
        package="vesc_ackermann",
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        parameters=[{'speed_to_erpm_gain' : 4614.0},
                    {'speed_to_erpm_offset': 0.0},
                    {'steering_angle_to_servo_gain': -1.2135},
                    {'steering_angle_to_servo_offset': 0.5304},
                   ],
        remappings=[('/ackermann_cmd', '/drive')]
    )


    # finalize
    ld.add_action(vehicle_control_node)
    ld.add_action(gap_follower_node)
    ld.add_action(joy_linux_node)
    ld.add_action(lidar_launchfile)
    ld.add_action(vesc_launchfile)
    ld.add_action(vesc_odometer)
    ld.add_action(vesc_ackermann)

    return ld
