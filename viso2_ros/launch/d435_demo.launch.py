#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, Shutdown

import time

from launch_ros.actions import Node
import os

'''


'''
    
def generate_launch_description():    
   
    ld = LaunchDescription()

    image_proc_dir = get_package_share_directory('image_proc')
    image_proc_launch_file = os.path.join(image_proc_dir, 'launch', 'image_proc.launch.py')
    image_proc_launcher = IncludeLaunchDescription( PythonLaunchDescriptionSource(image_proc_launch_file))
    
    # Find out what we need and use after
    #ld.add_action(image_proc_launcher)

    mono_odometer_node = Node(
                        package='viso2_ros', 
                        executable='mono_odometer', 
                        output='screen', 
                        remappings=[("/image", "/camera/color/image_raw"), 
                                    ("camera_info", "/camera/color/camera_info" )],
                        parameters=[{
                            "odom_frame_id": "odom",
                            "base_link_frame_id": "base_footprint",
                            "camera_height": 1.00, 
                            "camera_pitch": 0.00}],
                        arguments=['--ros-args', '--log-level', 'debug']
                    )
    ld.add_action(mono_odometer_node)

    return ld


#######################################

