from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from datetime import datetime
import os


def generate_launch_description():
    parameters = {
                "lidar_name": "lidar",
                "lidar_namespace": "", 
                'channel_type': 'serial',
                'serial_port': "/dev/ttyUSB0", 
                'serial_baudrate': 460800, 
                'frame_id': 'link',
                'inverted': False, 
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }

    def get_lidar(context):
        declared_parammeters = [
            dict(
                [
                    (
                        param_name, 
                        LaunchConfiguration(param_name)
                    ) 
                    for param_name, value in parameters.items()
                ]
            ), 
            {'base_frame_id': 'link'}]
        name = declared_parammeters[0]['lidar_name'].perform(context)
        namespace = declared_parammeters[0]['lidar_namespace'].perform(context)
        return [
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name=name,
                namespace=namespace,
                parameters=declared_parammeters
            )
        ]


    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(
                    param_name, 
                    default_value=str(param_value), 
                    description="") 
                    for param_name, param_value in parameters.items()
            ],
            OpaqueFunction(function=get_lidar)
        ]
    )
