#!/usr/bin/env python3
"""
Launch file for NMEA GPS to CAN Direct Converter
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for NMEA GPS to CAN converter"""
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GPS receiver'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Serial port baudrate'
    )
    
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value='can0',
        description='CAN channel name'
    )
    
    longitude_range_arg = DeclareLaunchArgument(
        'longitude_range',
        default_value='121.0',
        description='Expected longitude range for position validation'
    )
    
    latitude_range_arg = DeclareLaunchArgument(
        'latitude_range',
        default_value='25.0',
        description='Expected latitude range for position validation'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gps',
        description='Frame ID for GPS data'
    )
    
    # Create the NMEA GPS to CAN converter node
    nmea_gps_can_node = Node(
        package='gps_tracker',
        executable='nmea_gps_can',
        name='nmea_gps_can_converter',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'can_channel': LaunchConfiguration('can_channel'),
            'longitude_range': LaunchConfiguration('longitude_range'),
            'latitude_range': LaunchConfiguration('latitude_range'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        remappings=[
            # No remappings needed since we're sending directly to CAN
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        serial_baudrate_arg,
        can_channel_arg,
        longitude_range_arg,
        latitude_range_arg,
        frame_id_arg,
        
        # Nodes
        nmea_gps_can_node,
    ])
