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
                        executable='stereo_odometer',
                        name='stereo_odometer', 
                        output='screen', 
                        remappings=[('/info',  '/viso_info'),
                                    ('/odometry',  '/viso_odom'),
                                    ('/stereo_camera/left/camera_info',   '/camera/infra1/camera_info'),
                                    ('/stereo_camera/left/image',         '/camera/infra1/image_rect_raw'),
                                    ('/stereo_camera/right/camera_info',  '/camera/infra2/camera_info'),
                                    ('/stereo_camera/right/image',        '/camera/infra2/image_rect_raw')
                        ],
                        parameters=[{
                            "odom_frame_id": "viso_odom",
                            "base_link_frame_id": "camera_link" #"base_footprint"
                        }]
                    )
    ld.add_action(mono_odometer_node)

    return ld


#######################################

