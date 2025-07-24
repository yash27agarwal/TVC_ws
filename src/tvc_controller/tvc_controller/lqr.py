import numpy as np
import scipy.linalg

class LQRController: 
    def __init__(self, phy_params=None):
        """
        Initializes the LQR controller with system matrices A, B and placeholder cost matrices Q, R.
        
        Args:
            phy_params (dict, optional): Dictionary containing physical parameters.
                Required keys:
                - 'MASS': Vehicle mass in kg
                - 'G': Gravitational acceleration in m/s²
                - 'I_XX': Moment of inertia about X-axis in kg⋅m²
                - 'I_YY': Moment of inertia about Y-axis in kg⋅m²
                - 'I_ZZ': Moment of inertia about Z-axis in kg⋅m²
                - 'DIST_COM_2_THRUST': Distance from center of mass to thrust point in m
        
        Returns:
            bool: False if phy_params is None, otherwise initializes the controller
        
        Raises:
            AssertionError: If matrix dimensions are incorrect
        """
        if phy_params is None:
            raise ValueError("Physical parameters must be provided to initialize the LQR controller.")
        
        if not phy_params:
            raise ValueError("Physical parameters cannot be empty.")
        
        # Initialize system properties from physical parameters
        self.mass = phy_params['MASS']
        self.gravity = phy_params['G']
        self.I_XX = phy_params['I_XX']
        self.I_YY = phy_params['I_YY']
        self.I_ZZ = phy_params['I_ZZ']
        self.dist_com_2_thrust = phy_params['DIST_COM_2_THRUST']
        
        self.A = self.set_A()
        assert self.A.shape[0] == self.A.shape[1], "Matrix A must be square."
        assert self.A.shape[0] == 12, "Matrix A must be of size 12x12."
        
        self.B = self.set_B()
        assert self.B.shape == (12, 4), "Matrix B must be of size 12x4."
        
        self.Q = None
        self.R = None
        self.P = None
        self.K = None

    def solve_lqr(self) -> bool:
        """
        Solves the LQR problem and computes the optimal gain matrix K.
        
        Solves the continuous-time Algebraic Riccati Equation (ARE):
        A^T P + P A - P B R^(-1) B^T P + Q = 0
        
        Then computes the optimal feedback gain:
        K = R^(-1) B^T P
        
        Returns:
            bool: True if LQR solution successful, False otherwise
        
        Note:
            Updates self.P and self.K with the computed matrices.
            In case of failure, both matrices are set to None.
        """
        try:
            self.P = scipy.linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
            self.K = np.linalg.solve(self.R, self.B.T @ self.P)
            return True
        except Exception as e:
            print(f"Error solving LQR: {e}")
            self.K = None
            self.P = None
            return False

    def get_K(self) -> np.ndarray:
        """
        Returns the LQR gain matrix K.
        
        Returns:
            np.ndarray: The 4x12 LQR gain matrix K, or None if not computed
        """
        return self.K

    def get_P(self) -> np.ndarray:
        """
        Returns the solution P to the Riccati equation.
        
        Returns:
            np.ndarray: The 12x12 solution matrix P, or None if not computed
        """
        return self.P

    def set_A(self):
        """
        Constructs the state transition matrix A for the linearized quadrotor dynamics.
        
        The state vector is: [x, y, z, vx, vy, vz, qx, qy, qz, p, q, r]
        where:
        - x, y, z: Position in world frame (m)
        - vx, vy, vz: Velocity in world frame (m/s)
        - qx, qy, qz: Attitude quaternion vector part (dimensionless)
        - p, q, r: Angular velocity in body frame (rad/s)
        
        The linearization assumes small angles and uses simplified quaternion kinematics.
        
        Returns:
            np.ndarray: 12x12 system dynamics matrix A
        """
        return np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  # x_dot = vx
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # y_dot = vy
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],  # z_dot = vz
            [0, 0, 0, 0, 0, 0, 0, -2*self.gravity, 0, 0, 0, 0],  # vx_dot = -g*qy (small angle)
            [0, 0, 0, 0, 0, 0, 2*self.gravity, 0, 0, 0, 0, 0],   # vy_dot = g*qx (small angle)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # vz_dot = thrust/mass - g (handled in B matrix)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0, 0],  # qx_dot = p/2 (simplified quaternion kinematics)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0],  # qy_dot = q/2
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2],  # qz_dot = r/2
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # p_dot (handled in B matrix)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # q_dot (handled in B matrix)
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]   # r_dot (handled in B matrix)
        ])

    def set_B(self):
        """
        Constructs the control input matrix B for the linearized quadrotor dynamics.
        
        The control vector is: [qx_cmd, qy_cmd, thrust, r_cmd]
        where:
        - qx_cmd, qy_cmd: Desired attitude quaternion vector components (dimensionless)
        - thrust: Total thrust force in body z-direction (N)
        - r_cmd: Desired yaw rate (rad/s)
        
        The matrix maps control inputs to state derivatives considering:
        - Attitude control affects translational acceleration through gravity projection
        - Thrust directly affects vertical acceleration
        - Angular velocity commands create torques through control allocation
        
        Returns:
            np.ndarray: 12x4 control input matrix B
        """
        return np.array([
            [0, 0, 0, 0],  # x (no direct control)
            [0, 0, 0, 0],  # y (no direct control)
            [0, 0, 0, 0],  # z (no direct control)
            [0, -self.gravity, 0, 0],  # vx (affected by qy attitude command)
            [self.gravity, 0, 0, 0],   # vy (affected by qx attitude command)
            [0, 0, 1/self.mass, 0],    # vz (directly controlled by thrust)
            [0, 0, 0, 0],  # qx (no direct control, handled through angular dynamics)
            [0, 0, 0, 0],  # qy (no direct control, handled through angular dynamics)
            [0, 0, 0, 0],  # qz (no direct control, handled through angular dynamics)
            [-self.dist_com_2_thrust*self.mass*self.gravity/self.I_XX, 0, 0, 0],  # p (roll torque from qx)
            [0, -self.dist_com_2_thrust*self.mass*self.gravity/self.I_YY, 0, 0],  # q (pitch torque from qy)
            [0, 0, 0, 1/self.I_ZZ]     # r (direct yaw control)
        ])

    def set_Q(self, Q=None) -> bool:
        """
        Sets the state cost weighting matrix Q for the LQR cost function.
        
        The Q matrix penalizes deviations in the state variables. Larger values
        on the diagonal correspond to higher penalties for deviations in those states.
        
        Args:
            Q (np.ndarray, optional): 12x12 positive semi-definite state cost matrix.
                                     If None, returns False without setting.
        
        Returns:
            bool: True if Q is set successfully, False if Q is None
        
        Note:
            Q must be positive semi-definite for the LQR problem to be well-posed.
        """
        if Q is None:
            return False
        self.Q = Q
        return True

    def set_R(self, R=None) -> bool:
        """
        Sets the control cost weighting matrix R for the LQR cost function.
        
        The R matrix penalizes control effort. Larger values on the diagonal
        correspond to higher penalties for using those control inputs.
        
        Args:
            R (np.ndarray, optional): 4x4 positive definite control cost matrix.
                                     If None, returns False without setting.
        
        Returns:
            bool: True if R is set successfully, False if R is None
        
        Note:
            R must be positive definite for the LQR problem to have a unique solution.
        """
        if R is None:
            return False
        self.R = R
        return True


def main():
    lqr_controller = LQRController()
    K = lqr_controller.get_K()
    print("Gain Matrix K:")
    print(K)
    
    # Simple control test
    # State vector: [x, y, z, vx, vy, vz, qx, qy, qz, p, q, r]
    X_i = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # Initial state
    X_f = np.array([0, 1, -5, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # Final state
    X = X_i - X_f  # State error
    U = -K @ X  # Control input (negative feedback)
    print(f"\nControl Input U for X is: {U}")


if __name__ == "__main__":
    main()