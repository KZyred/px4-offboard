

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Load parameters from the YAML file
        self.declare_parameters(
            namespace='',
            parameters=[
                ('polygon_center', [0,0]),
                ('circum_radius',5),
                ('n_sides', 3),
                ('meter_per_sec', 0.5),
                ('polygon_angle', 0),
                ('uav_height', -5.0),
                ('uav_yaw', 0.0),
            ])

        # Retrieve parameters
        self.polygon_center = self.get_parameter('polygon_center').value
        self.circum_radius = self.get_parameter('circum_radius').value
        self.n_sides = self.get_parameter('n_sides').value
        self.meter_per_sec = self.get_parameter('meter_per_sec').value
        self.polygon_angle = self.get_parameter('polygon_angle').value
        self.uav_height = self.get_parameter('uav_height').value
        self.uav_yaw = self.get_parameter('uav_yaw').value
        # Automatically set
        self.speed_factor = 9.76 # Obtained by flight (trial and error)
        self.points_per_line = int(2*np.pi*self.circum_radius*self.speed_factor/(self.n_sides*self.meter_per_sec))
        self.counter = 0


        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)


        # Get full polygon path
        path = self.polygon_path(self.polygon_center, self.n_sides, self.circum_radius, self.points_per_line, self.polygon_angle)

        # Start sending setpoints if in offboard mode
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            # Trajectory setpoint - NED local world frame
            trajectory_msg = TrajectorySetpoint()
            # Timestamp is automatically set inside PX4
            trajectory_msg.timestamp = 0
            px, py = path[self.counter]
            pz = self.uav_height # in m
            trajectory_msg.position = [px, py, pz] # [x, y, z] in meters
            #trajectory_msg.velocity # in m/s
            #trajectory_msg.acceleration # in m/s^2
            #trajectory_msg.jerk # m/s^3 (for logging only)
            #trajectory_msg.yaw = self.uav_yaw # in degree
            #trajectory_msg.yawspeed = 0.0 # in rad/s

            # Publish
            self.publisher_trajectory.publish(trajectory_msg)

            # Control the path points counter
            self.counter+=1
            if self.counter == len(path):
                self.counter = 0
        else:
            self.counter = 0


    def polygon_path(self, polygon_center, n_sides, circum_radius, points_per_line, polygon_angle):
      # Get vertices
      theta = np.linspace(0, 2*np.pi, n_sides+1) + np.radians(polygon_angle)
      x = circum_radius * np.cos(theta) + polygon_center[0]
      y = circum_radius * np.sin(theta) + polygon_center[1]

      # Add points between vertices
      path_coordinates = []
      for i in range(n_sides):
          x_interp = np.linspace(x[i], x[i+1], points_per_line+2)[0:-1]
          y_interp = np.linspace(y[i], y[i+1], points_per_line+2)[0:-1]
          path_coordinates.extend(list(zip(x_interp, y_interp)))

      return path_coordinates


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()