#!/usr/bin/python3
 
from launch import LaunchDescription 

from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

'''


'''
    
def generate_launch_description():    
   
    ld = LaunchDescription()


    mono_odometer_node = Node(
                        package='viso2_ros', 
                        executable='mono_odometer', 
                        name='viso_odom_node',
                        output='screen', 
                        remappings=[("/image", "/camera/color/image_raw"), 
                                    ("camera_info", "/camera/color/camera_info" ),
                                    ("/odometry", "/visual_odom")],
                        parameters=[
                            PathJoinSubstitution([
                                get_package_share_directory("viso2_ros"),
                                "config", "config.yaml",
                            ])],
                        #arguments=['--ros-args', '--log-level', 'debug']
                    )
    ld.add_action(mono_odometer_node)

    return ld


#######################################

