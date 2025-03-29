from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name='drone_launch'
    pkg_share = get_package_share_directory(package_name)


    #todo
    return LaunchDescription([
        
    ])