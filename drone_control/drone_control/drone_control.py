import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
import tenseal as ts
import base64

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand

class DroneControl(Node):

    def __init__(self):
        super().__init__('drone_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter('system_id', 1)  
        self.system_id = self.get_parameter('system_id').value


        self.ts_context = None
        # self.ts_context = ts.context(
        #     ts.SCHEME_TYPE.CKKS,
        #     poly_modulus_degree=8192,
        #     coeff_mod_bit_sizes=[60, 40, 60]
        # )

        # self.ts_context.generate_galois_keys()
        # self.ts_context.global_scale = 2**40



        self.command_sub = self.create_subscription(
            String,
            f'drone_{self.system_id}/setpoint',
            self.command_callback,
            qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'/px4_{self.system_id}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            f'/px4_{self.system_id}/fmu/in/vehicle_command',
            qos_profile
        )    

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            f'/px4_{self.system_id}/fmu/in/offboard_control_mode', 
            qos_profile)
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            f'/px4_{self.system_id}/fmu/in/trajectory_setpoint',
            qos_profile)
        
        timer_period = 0.02  
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.last_arm_attempt = self.get_clock().now()
        self.arm_cooldown = 2.0 

        self.x = 3
        self.y = 3
        self.z = -15

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
    def command_callback(self, msg):
        try:
            if self.ts_context is None:
                try:
                    with open("ts_context.bin", "rb") as f:
                        context_bytes = f.read()
                    self.ts_context = ts.context_from(context_bytes)
                except FileNotFoundError:
                    self.ts_context = None
                    self.get_logger().info("file not found")
                    return

            serialized = base64.b64decode(msg.data)
            enc_vec = ts.ckks_vector_from(self.ts_context, serialized)
            decrypted = enc_vec.decrypt()

            # self.x, self.y, self.z = decrypted
            # if decrypted[0] < 100 and decrypted[0] > -100:
            #     self.x = decrypted[0]
            # if decrypted[1] < 100 and decrypted[1] > -100:
            #     self.y = decrypted[1]
            # if decrypted[2] < 100 and decrypted[2] > -100:
            #     self.x = decrypted[2]        
            self.x = decrypted[0]
            self.y = decrypted[1]
            self.z = decrypted[2]
            
            # self.get_logger().info(f"{self.x} {self.y} {self.z} = {decrypted[0]} {decrypted[1]} {decrypted[2]}")
        except Exception as e:
            self.get_logger().error(f"Decryption failed: {e}")


        # self.x = msg.position[0]
        # self.y = msg.position[1]
        # self.z = msg.position[2]

    def vehicle_status_callback(self, msg):
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info(f"{self.system_id} is not in Offboard mode")

            set_offboard_msg = VehicleCommand()
            set_offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            set_offboard_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            set_offboard_msg.param1 = 1.0  # Custom mode
            set_offboard_msg.param2 = 6.0  # PX4 Offboard
            set_offboard_msg.target_system = self.system_id + 1
            set_offboard_msg.target_component = 1
            set_offboard_msg.source_system = self.system_id + 1
            set_offboard_msg.source_component = 1
            set_offboard_msg.from_external = True

            self.publisher_vehicle_command.publish(set_offboard_msg)

        # Step 2: Arm only once Offboard mode is confirmed active
        elif self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info(f"{self.system_id} in Offboard but not armed")

            arm_msg = VehicleCommand()
            arm_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            arm_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_msg.param1 = 1.0  # Arm
            arm_msg.target_system = self.system_id + 1
            arm_msg.target_component = 1
            arm_msg.source_system = self.system_id + 1
            arm_msg.source_component = 1
            arm_msg.from_external = True

            if (self.get_clock().now() - self.last_arm_attempt).nanoseconds > 1e9 * self.arm_cooldown:
                self.publisher_vehicle_command.publish(arm_msg)
                self.last_arm_attempt = self.get_clock().now()

        # Step 3: Already armed + in Offboard → send trajectory
        else:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = self.x
            trajectory_msg.position[1] = self.y
            trajectory_msg.position[2] = self.z
            self.publisher_trajectory.publish(trajectory_msg)

        # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
        #     trajectory_msg = TrajectorySetpoint()
        #     trajectory_msg.position[0] = self.x
        #     trajectory_msg.position[1] = self.y
        #     trajectory_msg.position[2] = self.z
        #     self.publisher_trajectory.publish(trajectory_msg)
            
        # if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
        #     self.get_logger().info(f"{self.system_id} is not offboard")

        #     set_offboard_msg = VehicleCommand()
        #     set_offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        #     set_offboard_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        #     set_offboard_msg.param1 = 1.0
        #     set_offboard_msg.param2 = 6.0
        #     set_offboard_msg.target_system = self.system_id + 1
        #     set_offboard_msg.target_component = 1
        #     set_offboard_msg.source_system = self.system_id + 1
        #     set_offboard_msg.source_component = 1
        #     set_offboard_msg.from_external = True

        #     self.publisher_vehicle_command.publish(set_offboard_msg)

        # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:

        #     self.get_logger().info(f"{self.system_id} not armed")

        #     arm_msg = VehicleCommand()
        #     arm_msg.timestamp = int(Clock().now().nanoseconds / 1000 )
        #     arm_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        #     arm_msg.param1 = 1.0
        #     arm_msg.target_system = self.system_id + 1
        #     arm_msg.target_component = 1
        #     arm_msg.source_system = self.system_id + 1
        #     arm_msg.source_component= 1
        #     arm_msg.from_external = True

        #     if (self.get_clock().now() - self.last_arm_attempt).nanoseconds > 1e9 * self.arm_cooldown:
        #         self.publisher_vehicle_command.publish(arm_msg)
        #         self.last_arm_attempt = self.get_clock().now()

            # cmd = VehicleCommand()
            # cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_HOME

            # cmd.param1 = 1.0  # Use current position
            # cmd.param5 = 0.0  # Force latitude
            # cmd.param6 = 0.0  # Force longitude
            # cmd.param7 = 0.0  # Force altitude

            # self.publisher_vehicle_command.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    print("node main runnin")

    offboard_control = DroneControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

 