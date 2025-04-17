import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name='drone_launch'
    pkg_share = get_package_share_directory(package_name)

    config_path = os.path.join(pkg_share,'config','config.yaml')

    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    num_drones = config['drones']['num_drones']

    security_env = {
        'ROS_SECURITY_ENABLE': 'true',
        'ROS_SECURITY_STRATEGY': 'Permissive',  # Passive encryption
        'ROS_SECURITY_KEYSTORE': os.path.expanduser('~/sros2_demo/demo_keystore')
    }

    nodes = []
    for i in range(1,num_drones+1):
        nodes.append(
            Node(
                package = 'drone_control',
                executable = 'drone_control_node',
                name=f'drone_{i}',
                parameters=[{'system_id': i}]#,
                # env=security_env
            )
        )

    nodes.append(
        Node(
            package = 'drone_control',
            executable = 'central_control_node'#,
            # env=security_env
        )
    )

    # nodes.append(
        # Node(
            # package = 'drone_control',
            
        # )
    # )    
    
    return LaunchDescription(nodes)