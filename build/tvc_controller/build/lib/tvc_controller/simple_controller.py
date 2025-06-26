#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus, VehicleOdometry
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint
import numpy as np
from tvc_controller.lqr import LQRController 


# Desired states vector
# [x, y, z, x_dot, y_dot, z_dot, q_x, q_y, q_z, q_w, p, q, r]
X_F_TARGET = np.array([2., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.])

# Place holder for LQR gain matrix
K_LQR = None

class PX4Controller(Node):
    
    def __init__(self):
        super().__init__('px4_controller')
        
        # Configure QoS profile for compatibility with PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.thrust_setpoint_publisher = self.create_publisher(
            VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)
        self.torque_setpoint_publisher = self.create_publisher(
            VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', qos_profile)
        
        # Subscribers
        self.vehicle_odom_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        
        # Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_INIT
        self.offboard_setpoint_counter = 0
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.get_logger().info('PX4 Controller initialized')
    
    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscriber."""
        # Position in NED frame (m)
        self.current_position = np.array(msg.position)
        
        # Velocity in NED frame (m/s)
        self.current_velocity = np.array(msg.velocity)
        
        # Orientation as quaternion [qw, qx, qy, qz] from msg (Body to NED)
        q_msg = msg.q 
        
        # PX4 specific: "First value NaN if invalid/unknown"
        if np.isnan(q_msg[0]): 
            self.get_logger().warn("Odometry quaternion qw from msg is NaN. Using last valid NED quaternion.")
            self.current_quaternion = np.copy(self._last_update_quaternion) 
        else:
            # Reorder to [qx, qy, qz, qw] for internal use (Body-to-NED)
            current_q_ned = np.array([q_msg[1], q_msg[2], q_msg[3], q_msg[0]])
            if np.any(np.isnan(current_q_ned)):
                 self.get_logger().warn("Odometry quaternion (reordered) contains NaN. Using last valid NED quaternion.")
                 self.current_quaternion = np.copy(self._last_update_quaternion)
            else:
                 self.current_quaternion = current_q_ned
                 self._last_update_quaternion = np.copy(current_q_ned)
        
        # Angular velocity in body frame
        self.current_angular_velocity = np.array(msg.angular_velocity)

        if not self.is_state_initialized: 
            if np.any(np.isnan(self.current_position)) or \
               np.any(np.isnan(self.current_velocity)) or \
               np.any(np.isnan(self.current_quaternion)) or \
               np.any(np.isnan(self.current_angular_velocity)):
                self.get_logger().warn("Odometry data contains NaN values. State not initialized.")
                return
            self.is_state_initialized = True
            self.get_logger().info("Vehicle state initialized from odometry.")

        self._convert_NED_2_UNW()

    def _convert_NED_2_UNW(self):
        """Convert from NED to UNW coordinate frame"""
        # 1. Convert Position (NED to UNW)
        if self.current_position is not None:
            ned_pos = np.copy(self.current_position)
            self.current_position[0] = -ned_pos[2]   # UNW_U = -NED_D
            self.current_position[1] =  ned_pos[0]   # UNW_N =  NED_N
            self.current_position[2] = -ned_pos[1]   # UNW_W = -NED_E
            self._last_update_position = self.current_position.copy()

        # 2. Convert Velocity (NED to UNW)
        if self.current_velocity is not None:
            ned_vel = np.copy(self.current_velocity)
            self.current_velocity[0] = -ned_vel[2]   # UNW_vU = -NED_vD
            self.current_velocity[1] =  ned_vel[0]   # UNW_vN =  NED_vN
            self.current_velocity[2] = -ned_vel[1]   # UNW_vW = -NED_vE
            self._last_update_velocity = self.current_velocity.copy()

        # 3. Convert Orientation Quaternion (Body to NED -> Body to UNW)
        if self.current_quaternion is not None:
            q_body_to_ned_xyzw = self.current_quaternion
            r_body_to_ned = R.from_quat(q_body_to_ned_xyzw)

            R_ned_to_unw_matrix = np.array([
                [0,  0, -1],
                [1,  0,  0],
                [0, -1,  0]
            ])
            r_ned_to_unw = R.from_matrix(R_ned_to_unw_matrix)
            r_body_to_unw = r_ned_to_unw * r_body_to_ned
            
            self.current_quaternion = r_body_to_unw.as_quat()
            self._last_update_quaternion = np.copy(self.current_quaternion)

        # 4. Angular Velocity (remains in body frame)
        self._last_update_angular_velocity = np.copy(self.current_angular_velocity)
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
    
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
    
    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
    
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')
    
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = True  # Enable thrust and torque control
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_thrust_setpoint(self, thrust_x=0.0, thrust_y=0.0, thrust_z=0.0):
        """Publish thrust setpoint in body frame."""
        msg = VehicleThrustSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.xyz = [thrust_x, thrust_y, thrust_z]  # Thrust in body frame [N]
        self.thrust_setpoint_publisher.publish(msg)
    
    def publish_torque_setpoint(self, torque_x=0.0, torque_y=0.0, torque_z=0.0):
        """Publish torque setpoint in body frame."""
        msg = VehicleTorqueSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.xyz = [torque_x, torque_y, torque_z]  # Torque in body frame [Nm]
        self.torque_setpoint_publisher.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
        return
    
    def _cal_quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1[3], q1[0], q1[1], q1[2]
        w2, x2, y2, z2 = q2[3], q2[0], q2[1], q2[2]
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return np.array([x, y, z, w])
    
    def _cal_quaternion_inverse(self, q):
        """Get quaternion inverse"""
        return np.array([-q[0], -q[1], -q[2], q[3]])
    
    def _cal_quaternion_error(self, q_current, q_target):
        """Calculate quaternion error"""
        q_diff = self._cal_quaternion_multiply(
            q_current, self._cal_quaternion_inverse(q_target))
        return q_diff
    
    def timer_callback(self):
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        # Check if armed
        if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
            if self.offboard_setpoint_counter < 50:
                self.get_logger().info(f'Vehicle armed! Counter: {self.offboard_setpoint_counter}')
            
            # Calculate state error
            pos_error = self._last_update_position - X_F_TARGET[:3]
            vel_error = self._last_update_velocity - X_F_TARGET[3:6]
            quaternion_error = self._cal_quaternion_error(
                self._last_update_quaternion, X_F_TARGET[6:10])
            angular_velocity_error = self._last_update_angular_velocity - X_F_TARGET[10:13]

            # Combine all errors
            x_error = np.zeros(13)
            x_error[:3] = pos_error
            x_error[3:6] = vel_error
            x_error[6:10] = quaternion_error
            x_error[10:13] = angular_velocity_error

            # LQR Control Law
            u_lqr = -K_LQR @ x_error
            
            # Publish thrust and torque setpoints
            self.publish_thrust_setpoint(u_lqr[0], u_lqr[1], u_lqr[2])
            self.publish_torque_setpoint(0.0, 0.0, 0.0)
            
            if self.offboard_setpoint_counter % 50 == 0:  # Log every 5 seconds
                self.get_logger().info(
                    f'Thrust: [{u_lqr[0]:.2f}, {u_lqr[0]:.2f}, {u_lqr[0]:.2f}] N, '
                    f'Torque: [{0}, {0}, {0}] Nm')
        
        elif self.arm_state == VehicleStatus.ARMING_STATE_DISARMED:
            if self.offboard_setpoint_counter < 50:
                self.get_logger().info(f'Vehicle not armed. Arming state: {self.arm_state}')
        
        if self.offboard_setpoint_counter < 11:
            # Publish neutral setpoints before arming
            self.publish_thrust_setpoint(1e-8, 0.0, 0.0)
            self.publish_torque_setpoint(0.0, 0.0, 0.0)
        
        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:

    lqr_controller = LQRController()
    lqr_controller.solve_lqr()
    K_LQR = lqr_controller.get_K()
    
    if K_LQR is None:
        raise ValueError("LQR gain matrix K is not defined. Please check your LQR setup.")
    
    if K_LQR.shape != (3, 13):
        raise ValueError(f"LQR gain matrix K has an unexpected shape: {K_LQR.shape}. Expected (3, 13).")
    print("LQR gain matrix K successfully calculated!")


    print('Starting PX4 Controller...')
    rclpy.init(args=args)
    px4_controller = PX4Controller()
    rclpy.spin(px4_controller)
    px4_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)