import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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

        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            qos_profile
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            'fmu/in/offboard_control_mode', 
            qos_profile)
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos_profile)
        
        timer_period = 0.02  
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        

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
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = 3
            trajectory_msg.position[1] = 3
            trajectory_msg.position[2] = -5
            self.publisher_trajectory.publish(trajectory_msg)
            
        elif self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            set_offboard_msg = VehicleCommand()
            set_offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            set_offboard_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            set_offboard_msg.param1 = 1.0
            set_offboard_msg.param2 = 6.0
            set_offboard_msg.target_system = 2
            set_offboard_msg.target_component = 1
            set_offboard_msg.source_system = 2 #experiment with 1
            set_offboard_msg.source_component = 1
            set_offboard_msg.from_external = True

            self.publisher_vehicle_command.publish(set_offboard_msg)

        elif self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:

            arm_msg = VehicleCommand()
            arm_msg.timestamp = int(Clock().now().nanoseconds / 1000 )
            arm_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            arm_msg.param1 = 1.0
            arm_msg.target_system = 2
            arm_msg.target_component = 1
            arm_msg.source_system = 2
            arm_msg.source_component= 1
            arm_msg.from_external = True

            self.publisher_vehicle_command.publish(arm_msg)



def main(args=None):
    rclpy.init(args=args)
    print("node main runnin")

    offboard_control = DroneControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()
0

if __name__ == '__main__':
    main()

 