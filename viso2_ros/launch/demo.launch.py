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

def launch_setup(context, *args, **kwargs):   

    # camera namespace
    ns = LaunchConfiguration('namespace').perform(context)

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
                        namespace=ns,
                        remappings=[(f"{ns}/image", f"{ns}/image_raw")],
                        parameters=[{
                            "odom_frame_id": "odom",
                            "base_link_frame_id": "camera",
                            "camera_height": 1.00, 
                            "camera_pitch": 0.00}]
                    )
    ld.add_action(mono_odometer_node)

    return [ld]
    
def generate_launch_description():    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace', 
            default_value='',
            description='camera namespace'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        OpaqueFunction(function = launch_setup)
        ])    


#######################################

