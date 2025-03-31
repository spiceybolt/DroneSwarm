import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name='drone_launch'
    pkg_share = get_package_share_directory(package_name)

    config_path = os.path.join(pkg_share,'config','config.yaml')

    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    num_drones = config['drones']['num_drones']
    pos = config['drones']['start_positions']

    delay = 7.0

    processes = []
    for i in range(1,num_drones+1):
        proc = ExecuteProcess(
                cmd = [f"PX4_SYS_AUTOSTART=4010 PX4_SIM_MODEL=gz_x500_mono_cam PX4_GZ_MODEL_POSE=\"{pos[i-1]['x']},{pos[i-1]['y']}, 0\" \
                                 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i {i}"],
                shell=True
            )
        #immediately loading them all at once causes them to crash 
        processes.append(
            TimerAction(
                period = delay * i,
                actions=[proc]
            )
        )

    ros2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'drones.launch.py')
        )
    )

    #some bug affects z target of the drone. not sure what.
    delayed_launch = TimerAction(
        period = delay*(num_drones+1),
        actions=[ros2_launch]
    )


    return LaunchDescription(processes + [delayed_launch])