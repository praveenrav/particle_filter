
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    params_file_dec = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('particle_filter'),'config','pf_params.yaml'),
        description='Full path to params file for particle filter.')
    
    params_path = '/home/praveenrav702/dev_ws/src/particle_filter/config/pf_params.yaml'

    pf_node = Node(
            package='particle_filter',
            executable='pf',
            parameters=[params_path]
         )

    return LaunchDescription([
        params_file_dec,
        pf_node,
    ])