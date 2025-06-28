#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus, VehicleOdometry
from px4_msgs.msg import VehicleThrustSetpoint, VehicleTorqueSetpoint, ArmingCheckRequest, ArmingCheckReply
import numpy as np
from tvc_controller.lqr import LQRController 

"""
Coordinate Convention:
- NED (North, East, Down) frame is used for position and velocity.
- FRD (Forward, Right, Down) frame is used for angular velocity.
"""

# Desired states vector - target orientation is now always [0, 0, 0, 1]
# [x, y, z, x_dot, y_dot, z_dot, q_x, q_y, q_z, q_w, p, q, r]
X_F_TARGET = np.array([0., 0., -5., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.])

class PX4Controller(Node):
    
    def __init__(self, K_LQR=None):
        super().__init__('px4_controller')
        
        # Configure QoS profile for compatibility with PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.K_LQR = K_LQR
        
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
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.is_state_initialized = False
        self.is_final_state_initialized = False
        self.offboard_setpoint_counter = 0

        self.state_callback_counter = 0
        # Initialize as empty lists to collect measurements
        self._list_of_positions = []
        self._list_of_quaternions = []

        self._last_update_position = None
        self._last_update_velocity = None
        self._last_update_quaternion = None
        self._last_update_angular_velocity = None
        
        # NEW: Variables for orientation normalization
        self.initial_rotation_matrix = None  # Fixed rotation matrix calculated at startup
        self.is_orientation_normalized = False
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.control_time = self.create_timer(0.01, self.control_timer)  # 100Hz
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
            if self._last_update_quaternion is None:
                self.get_logger().warn("No last valid quaternion available. Using default quaternion [0, 0, 0, 1].")
                self.current_quaternion = np.array([0, 0, 0, 1])
            else:
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
        
        # Angular velocity in body frame (FRD)
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

        # self._convert_NED_2_UNW()
        self._pre_precess_date()

        # Collect measurements for averaging (first 100 readings)
        if self.state_callback_counter < 100:
            if self._last_update_position is not None and self._last_update_quaternion is not None:
                self._list_of_positions.append(self._last_update_position.copy())
                self._list_of_quaternions.append(self._last_update_quaternion.copy())

        if self.state_callback_counter == 100:
            self._initialize_final_state()
        
        self.state_callback_counter += 1

    def _pre_precess_date(self):
        """Pre-process data to convert from NED to UNW coordinate frame."""
        if self.current_position is not None:
            # self.get_logger().info(f"Current position (NED frame): {self.current_position}")
            self._last_update_position = self.current_position.copy()

        if self.current_velocity is not None:
            # self.get_logger().info(f"Current velocity (NED frame): {self.current_velocity}")
            self._last_update_velocity = self.current_velocity.copy()

        if self.current_quaternion is not None:
            # self.get_logger().info(f"Current quaternion (NED frame): {self.current_quaternion}")
            
            if self.is_orientation_normalized:
                # Apply the fixed rotation to normalize orientation
                r_current = R.from_quat(self.current_quaternion)
                r_normalized = R.from_matrix(self.initial_rotation_matrix) * r_current
                self.current_quaternion = r_normalized.as_quat()
                # self.get_logger().info(f"Normalized quaternion (NED frame): {self.current_quaternion}")
                
                # Ensure the scalar part is positive
                if self.current_quaternion[3] < 0:
                    self.get_logger().warn("Quaternion scalar part is negative!, switching signs.")
                    self.current_quaternion = -self.current_quaternion

            self._last_update_quaternion = self.current_quaternion.copy()

        if self.current_angular_velocity is not None:
            # self.get_logger().info(f"Current angular velocity (NED frame): {self.current_angular_velocity}")
            self._last_update_angular_velocity = self.current_angular_velocity.copy()

    def _initialize_orientation_normalization(self, avg_quaternion):
        """Initialize the fixed rotation matrix for orientation normalization."""
        # Calculate the rotation that transforms avg_quaternion to [0, 0, 0, 1]
        # We want: R_norm * R_avg = I (identity rotation)
        # Therefore: R_norm = R_avg^(-1)
        
        r_avg = R.from_quat(avg_quaternion)
        r_norm = r_avg.inv()  # Inverse rotation
        self.initial_rotation_matrix = r_norm.as_matrix()
        
        self.get_logger().info(f"Orientation normalization matrix calculated:")
        self.get_logger().info(f"Initial rotation matrix:\n{self.initial_rotation_matrix}")
        
        # Verify: applying this rotation to avg_quaternion should give [0,0,0,1]
        r_test = r_norm * r_avg
        q_test = r_test.as_quat()
        
        # Ensure positive scalar part for the verification
        if q_test[3] < 0:
            self.get_logger().error("Quaternion scalar part is negative!, something is wrong.")
            
        self.get_logger().info(f"Verification - normalized quaternion: {q_test}")
        
        self.is_orientation_normalized = True

    def _initialize_final_state(self):
        """Initialize the final state for LQR control."""
        if self.is_final_state_initialized:
            return
        
        if len(self._list_of_positions) == 0 or len(self._list_of_quaternions) == 0:
            self.get_logger().error("No position or quaternion data collected for averaging!")
            return
            
        # Convert lists to numpy arrays for averaging
        positions_array = np.array(self._list_of_positions)  # Shape: (n_samples, 3)
        quaternions_array = np.array(self._list_of_quaternions)  # Shape: (n_samples, 4)
        
        # Calculate averages
        _avg_position = np.mean(positions_array, axis=0)  # Shape: (3,)
        _avg_quaternion = np.mean(quaternions_array, axis=0)  # Shape: (4,)
        
        # Normalize the averaged quaternion
        _avg_quaternion = _avg_quaternion / np.linalg.norm(_avg_quaternion)
        
        self.get_logger().info(f"Average Position: {_avg_position}")
        self.get_logger().info(f"Average Quaternion (before normalization): {_avg_quaternion}")
        
        # NEW: Initialize orientation normalization
        self._initialize_orientation_normalization(_avg_quaternion)
        
        # Set target position by adding the desired offset to average position
        self.position_target = np.array([
            _avg_position[0] + X_F_TARGET[0], 
            _avg_position[1] + X_F_TARGET[1], 
            _avg_position[2] + X_F_TARGET[2]
        ])

        # NEW: Target quaternion is always [0, 0, 0, 1] in normalized frame
        self.quaternion_target = np.array([X_F_TARGET[6], X_F_TARGET[7], X_F_TARGET[8], X_F_TARGET[9]])
        
        # Set target velocities and angular velocities
        self.velocity_target = np.array([X_F_TARGET[3], X_F_TARGET[4], X_F_TARGET[5]])
        self.angular_velocity_target = np.array([X_F_TARGET[10], X_F_TARGET[11], X_F_TARGET[12]])

        self.get_logger().info(f"Target Position: {self.position_target}")
        self.get_logger().info(f"Target Quaternion (normalized frame): {self.quaternion_target}")
        self.get_logger().info(f"Target Velocity: {self.velocity_target}")
        self.get_logger().info(f"Target Angular Velocity: {self.angular_velocity_target}")

        self.is_final_state_initialized = True
        self.get_logger().info("Final state initialized for LQR control with normalized orientation.")
        self.get_logger().warn("__________________________________________________________\n")

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
    
    def publish_thrust_setpoint(self, thrust_sp = np.array([0, 0, 0])):
        """Publish thrust setpoint in body frame."""
        msg = VehicleThrustSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.xyz = [thrust_sp[0], thrust_sp[1], thrust_sp[2]]  # Thrust in body frame [N]
        self.thrust_setpoint_publisher.publish(msg)
    
    def publish_torque_setpoint(self, torque_sp_roll = 0):
        """Publish torque setpoint in body frame."""
        msg = VehicleTorqueSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.xyz = [float(torque_sp_roll), 0.0, 0.0]  # Torque in body frame [Nm]
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
        
        if self.offboard_setpoint_counter == 10:
            self.arm()

        if self.offboard_setpoint_counter == 25:
            self.engage_offboard_mode()
        
        if self.offboard_setpoint_counter < 5000:
            self.offboard_setpoint_counter += 1

    
    def control_timer(self):
        self.publish_offboard_control_heartbeat_signal()

        # (TODO):Add a check for armed and in offboard mode
        if self.is_state_initialized and self.is_final_state_initialized:
            
            # Calculate state error
            pos_error = self._last_update_position - self.position_target
            vel_error = self._last_update_velocity - self.velocity_target
            quaternion_error = self._cal_quaternion_error(
                self._last_update_quaternion, self.quaternion_target)
            angular_velocity_error = self._last_update_angular_velocity - self.angular_velocity_target

            # Combine all errors
            x_error = np.zeros(12)
            x_error[:3] = pos_error
            x_error[3:6] = vel_error
            x_error[6:9] = quaternion_error[:3]
            x_error[9:12] = angular_velocity_error

            # LQR Control Law
            u_lqr = -self.K_LQR @ x_error
            thrust_sp = u_lqr[0:3]
            torque_sp = float(u_lqr[3])

            # Add hover thrust in the z-direction (NED frame) 
            thrust_sp[2] -= 0.567 * 9.81  
            
            self.get_logger().info(f'Control Output: {u_lqr}')

            max_thrust = 9.81 * 1.4  # Maximum thrust limit in N
            if np.linalg.norm(thrust_sp) > max_thrust:
                self.get_logger().warn(f'Thrust setpoint 0 exceeds max limit: {np.linalg.norm(thrust_sp)} N')
                thrust_sp_dir = thrust_sp / np.linalg.norm(thrust_sp)\
                      if np.linalg.norm(thrust_sp) > 0 else np.zeros(3)
                thrust_sp = thrust_sp_dir * max_thrust

            # if thrust_sp[0] < -max_thrust*np.sin(30 * np.pi / 180):
            #     self.get_logger().warn(f'Thrust setpoint 0 in x-axis is negative: {thrust_sp[0]} N, setting to 0')
            #     thrust_sp[0] = -max_thrust * np.sin(30 * np.pi / 180)
            
            # if thrust_sp[0] > max_thrust * np.sin(30 * np.pi / 180):
            #     self.get_logger().warn(f'Thrust setpoint 1 in x-axis is exceeds max limit: {thrust_sp[0]} N')
            #     thrust_sp[0] = max_thrust * np.sin(30 * np.pi / 180)

            # if thrust_sp[1] < -max_thrust*np.sin(30 * np.pi / 180):
            #     self.get_logger().warn(f'Thrust setpoint in x-axis is negative: {thrust_sp[1]} N, setting to 0')
            #     thrust_sp[1] = -max_thrust * np.sin(30 * np.pi / 180)
            
            # if thrust_sp[1] > max_thrust * np.sin(30 * np.pi / 180):
            #     self.get_logger().warn(f'Thrust setpoint in x-axis is positive: {thrust_sp[1]} N, setting to 0')
            #     thrust_sp[1] = max_thrust * np.sin(30 * np.pi / 180)

            
            if thrust_sp[2] > 0 :
                self.get_logger().warn(f'Thrust setpoint in axis is negative: {thrust_sp[2]} N, setting to 0')
                thrust_sp[2] = 0.0
                # thrust_sp_dir = thrust_sp / np.linalg.norm(thrust_sp)\
                #       if np.linalg.norm(thrust_sp) > 0 else np.zeros(3)
                # thrust_sp = thrust_sp_dir * 9.81 * 0.6

            # Publish thrust and torque setpoints
            self.publish_thrust_setpoint(thrust_sp)
            self.publish_torque_setpoint(torque_sp)

def main(args=None) -> None:

    lqr_controller = LQRController()
    lqr_controller.solve_lqr()
    K_LQR = lqr_controller.get_K()
    
    if K_LQR is None:
        raise ValueError("LQR gain matrix K is not defined. Please check your LQR setup.")
    if K_LQR.shape != (4, 12):
        raise ValueError(f"LQR gain matrix K has an unexpected shape: {K_LQR.shape}. Expected (4, 12).")
    print("LQR gain matrix K successfully calculated!")

    print('Starting PX4 Controller...')
    rclpy.init(args=args)
    px4_controller = PX4Controller(K_LQR=K_LQR)
    rclpy.spin(px4_controller)
    px4_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)