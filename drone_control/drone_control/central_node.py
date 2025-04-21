import os
import yaml
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
import tenseal as ts
import base64
import json
import math

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

        self.ts_context = ts.context(
            ts.SCHEME_TYPE.CKKS,
            poly_modulus_degree=8192,
            coeff_mod_bit_sizes=[60, 40, 60]
        )
        self.ts_context.generate_galois_keys()
        self.ts_context.global_scale = 2**40

        self.context_bytes = self.ts_context.serialize(save_secret_key=True)

        with open("ts_context.bin", "wb") as f:
            f.write(self.context_bytes)

        self.publishers_list = []

        for i in range(self.num_drones):
            self.publishers_list.append(self.create_publisher(
                String,
                f'drone_{i+1}/setpoint',
                qos_profile
            ))
        
        self.time_period = 0.01
        self.timer = self.create_timer(self.time_period, self.publish)

        self.radius = 5.0
        self.angular_speed = 0.2  # radians per second
        self.altitude = -15.0     # Fixed z altitude

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.ypos = 0.0
        self.coeff = 1

        self.alt = -15.0

    def publish(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        t = current_time - self.start_time
        angle = self.angular_speed * t

        for i in range(self.num_drones):

            offset_x = self.pos[i]['x']
            offset_y = self.pos[i]['y']

            center_x = self.radius * math.cos(angle)
            center_y = self.radius * math.sin(angle)

            # Maintain formation relative to circular center
            target_x = center_x + offset_x
            target_y = center_y + offset_y
            target_z = self.altitude

            # vec = [3 * i, self.ypos, self.alt]
            vec = [target_x, target_y, target_z]
            enc_vec = ts.ckks_vector(self.ts_context, vec)
            serialized = enc_vec.serialize()
            encoded = base64.b64encode(serialized).decode('utf-8')

            msg = String()
            msg.data = encoded
            self.publishers_list[i].publish(msg)


            msg = String()
        #     msg = TrajectorySetpoint()
        #     msg.position[0] = 3 * i
        #     msg.position[1] =  self.ypos
        #     msg.position[2] = self.alt

        #     self.publishers_list[i].publish(msg)
        
        # self.ypos = self.ypos + self.time_period * self.coeff

        # if self.ypos >= 10.0 or self.ypos <= -10.0:
        #     self.coeff = self.coeff * -1
    
        
    

def main(args=None):
    rclpy.init(args=args)

    central_control = CentralControl()

    rclpy.spin(central_control)

    central_control.destroy_node()
    rclpy.shutdown()
    file_path = "ts_context.bin"
    if os.path.exists(file_path):
        os.remove(file_path)
        print(f"{file_path} has been deleted.")
    else:
        print(f"{file_path} does not exist.")


if __name__ == '__main__':
    main()

 