import os
import yaml
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from px4_msgs.msg import TrajectorySetpoint

class CentralControl(Node):

    def __init__(self):
        super().__init__('central_control_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        config_path = os.path.join(get_package_share_directory('drone_launch'),'config','config.yaml')

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.num_drones = config['drones']['num_drones']
        self.pos = config['drones']['start_positions']

        self.publishers_list = []

        for i in range(self.num_drones):
            self.publishers_list.append(self.create_publisher(
                TrajectorySetpoint,
                f'drone_{i+1}/setpoint',
                qos_profile
            ))
        
        self.time_period = 0.01
        self.timer = self.create_timer(self.time_period, self.publish)

        self.ypos = 0.0
        self.coeff = 1

        self.alt = -5.0

    def publish(self):
        for i in range(self.num_drones):

            msg = TrajectorySetpoint()
            msg.position[0] = 3 * i
            msg.position[1] =  self.ypos
            msg.position[2] = self.alt

            self.publishers_list[i].publish(msg)
        
        self.ypos = self.ypos + self.time_period * self.coeff

        if self.ypos >= 10.0 or self.ypos <= -10.0:
            self.coeff = self.coeff * -1
    
        
    

def main(args=None):
    rclpy.init(args=args)

    central_control = CentralControl()

    rclpy.spin(central_control)

    central_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

 