import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

class DroneControl(Node):

    def __init__(self):
        super().__init__('drone_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
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
        
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value

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
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = 3
            trajectory_msg.position[1] = 3
            trajectory_msg.position[2] = -5
            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    print("node main runnin")

    offboard_control = DroneControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

 