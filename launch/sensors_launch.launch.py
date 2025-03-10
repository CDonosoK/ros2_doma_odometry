from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('ros2_doma_odometry'),
        'config',
        'config.yaml'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    return LaunchDescription(
        generate_nodes(config)
    )

def generate_nodes(config_file):
    camera_node = Node(
        package='ros2_doma_odometry',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    gps_node = Node(
        package='ros2_doma_odometry',
        executable='gps_node',
        name='gps_node',
        output='screen',
    )

    imu_node = Node(
        package='ros2_doma_odometry',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )

    return [camera_node, gps_node, imu_node]