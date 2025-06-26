import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

G = 9.81
DIST_COM = -1.1087 
I_XX = 0.001403
I_YY = 0.111939 
I_ZZ = 0.112119
MASS = 0.567    

class LQRController:
    def __init__(self):
        """
        Initializes the LQR controller with system matrices A, B and cost matrices Q, R.
        """
        self.A = self.set_A()
        assert self.A.shape[0] == self.A.shape[1], "Matrix A must be square."
        assert self.A.shape[0] == 12, "Matrix A must be of size 12x12."

        self.B = self.set_B()
        assert self.B.shape == (12, 4), "Matrix B must be of size 12x4."

        self.Q = self.set_Q()
        self.R = self.set_R()
        self.P = None
        self.K = None

        # State and input names for analysis
        self.state_names = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'qx', 'qy', 'qz', 'p', 'q', 'r']
        self.input_names = ['Fx', 'Fy', 'Fz', 'Mx']

    def solve_lqr(self):
        """Solves the LQR problem and returns the gain matrix K and solution P."""
        self.P = scipy.linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.solve(self.R, self.B.T @ self.P)

    def get_K(self):
        return self.K
    
    def get_P(self):
        return self.P

    def set_A(self):
        """Constructs the state transition matrix A for the system dynamics."""
        return np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],       # x_dot
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],       # y_dot
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],     # z_dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],       # vx_dot 
            [0, 0, 0, 0, 0, 0, 0, 0, 2*G, 0, 0, 0],     # vy_dot
            [0, 0, 0, 0, 0, 0, 0, -2*G, 0, 0, 0, 0],        # vz_dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0, 0],      # qx_dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0],     # qy_dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2],     # qz_dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],       # p_dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],       # q_dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]    # r_dot
        ])
    
    def set_B(self):
        return np.array([ 
            [0, 0, 0, 0],       # x
            [0, 0, 0, 0],       # y
            [0, 0, 0, 0],    # z
            [1/MASS, 0, 0, 0],  # vx
            [0, 1/MASS, 0, 0],  # vy
            [0, 0, 1/MASS, 0],  # vz
            [0, 0, 0, 0],   # qx
            [0, 0, 0, 0],       # qy
            [0, 0, 0, 0],   # qz
            [0, 0, 0, 1/I_XX],  # p
            [0, 0, -DIST_COM/I_YY, 0],      # q
            [0, DIST_COM/I_ZZ, 0, 0]        # r
        ])
    
    def set_Q(self):
        return np.diag([5, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 0.001, 50, 50, 0.001, 10, 10])
    
    def set_R(self):
        return np.diag([1, 1, 1, 1])
    
    def get_eigenvalues(self):
        """Returns the eigenvalues of the system matrix A."""
        return np.linalg.eigvals(self.A)

    # ========== VERIFICATION TESTS ==========
    
    def check_controllability(self):
        """Check if the system (A,B) is controllable"""
        n = self.A.shape[0]
        # Controllability matrix: [B AB A²B ... A^(n-1)B]
        controllability_matrix = self.B.copy()
        
        A_power = np.eye(n)  # Start with identity
        for i in range(1, n):
            A_power = A_power @ self.A
            controllability_matrix = np.hstack([controllability_matrix, A_power @ self.B])
        
        rank = np.linalg.matrix_rank(controllability_matrix)
        is_controllable = rank == n
        
        print(f"Controllability matrix rank: {rank}/{n}")
        print(f"System is controllable: {is_controllable}")
        
        if not is_controllable:
            print("WARNING: System is not fully controllable!")
            # Find uncontrollable modes
            U, s, Vh = np.linalg.svd(controllability_matrix)
            print(f"Singular values: {s}")
            
        return is_controllable

    def check_closed_loop_stability(self):
        """Check if closed-loop system A-BK is stable"""
        if self.K is None:
            print("ERROR: Solve LQR first!")
            return False, None
            
        A_cl = self.A - self.B @ self.K
        eigenvalues = np.linalg.eigvals(A_cl)
        
        print("Closed-loop eigenvalues:")
        for i, eig in enumerate(eigenvalues):
            if np.abs(np.imag(eig)) < 1e-10:  # Real eigenvalue
                print(f"  λ{i+1}: {np.real(eig):8.6f}")
            else:  # Complex eigenvalue
                print(f"  λ{i+1}: {np.real(eig):8.6f} + {np.imag(eig):8.6f}j")
        
        # Check stability (all real parts < 0)
        real_parts = np.real(eigenvalues)
        is_stable = np.all(real_parts < 0)
        max_real_part = np.max(real_parts)
        
        print(f"Maximum real part: {max_real_part:.6f}")
        print(f"System is stable: {is_stable}")
        
        if not is_stable:
            unstable_modes = np.where(real_parts >= 0)[0]
            print(f"WARNING: Unstable modes at indices: {unstable_modes}")
        
        return is_stable, eigenvalues

    def verify_are_solution(self):
        """Verify P satisfies the ARE: A^T P + P A - P B R^(-1) B^T P + Q = 0"""
        if self.P is None:
            print("ERROR: Solve LQR first!")
            return False
        
        # Compute ARE residual
        R_inv = np.linalg.inv(self.R)
        residual = (self.A.T @ self.P + self.P @ self.A - 
                    self.P @ self.B @ R_inv @ self.B.T @ self.P + self.Q)
        
        residual_norm = np.linalg.norm(residual, 'fro')
        print(f"ARE residual norm: {residual_norm:.2e}")
        
        # Should be close to machine precision
        tolerance = 1e-10
        is_valid = residual_norm < tolerance
        print(f"ARE solution is valid (< {tolerance:.0e}): {is_valid}")
        
        if not is_valid:
            print("WARNING: ARE solution may be inaccurate!")
            print(f"Max residual element: {np.max(np.abs(residual)):.2e}")
        
        return is_valid

    def analyze_gains(self):
        """Analyze the physical meaning of control gains"""
        if self.K is None:
            print("ERROR: Solve LQR first!")
            return
        
        print("\n" + "="*80)
        print("CONTROL GAIN MATRIX K ANALYSIS")
        print("="*80)
        print("Control law: u = -K * (x - x_ref)")
        print("Rows: inputs, Columns: states")
        print()
        
        # Print header
        header = f"{'Input':<8} " + " ".join(f'{name:>8}' for name in self.state_names)
        print(header)
        print("-" * len(header))
        
        # Print gain matrix with labels
        for i, input_name in enumerate(self.input_names):
            gains_str = " ".join(f'{self.K[i,j]:8.3f}' for j in range(len(self.state_names)))
            print(f"{input_name:<8} {gains_str}")
        
        print("\nKey Physical Interpretations:")
        print(f"Fx vs x position:  {self.K[0,0]:8.3f} (position feedback)")
        print(f"Fx vs x velocity:  {self.K[0,3]:8.3f} (damping)")
        print(f"Fz vs z position:  {self.K[2,2]:8.3f} (altitude control)")
        print(f"Fz vs z velocity:  {self.K[2,5]:8.3f} (altitude damping)")
        print(f"Mx vs roll angle:  {self.K[3,6]:8.3f} (attitude control)")
        print(f"Mx vs roll rate:   {self.K[3,9]:8.3f} (rate damping)")
        
        # Check for unexpected large cross-couplings
        print("\nCross-coupling Analysis:")
        K_abs = np.abs(self.K)
        for i in range(self.K.shape[0]):
            max_gain = np.max(K_abs[i, :])
            dominant_states = np.where(K_abs[i, :] > 0.1 * max_gain)[0]
            dominant_names = [self.state_names[j] for j in dominant_states]
            print(f"{self.input_names[i]} mainly controlled by: {', '.join(dominant_names)}")

    def simulate_step_response(self, x0, xf, t_final=10, dt=0.01):
        """Simulate closed-loop step response"""
        if self.K is None:
            print("ERROR: Solve LQR first!")
            return None, None, None
        
        # Time vector
        t = np.arange(0, t_final, dt)
        n_steps = len(t)
        n_states = len(x0)
        n_inputs = self.B.shape[1]
        
        # Storage
        x_traj = np.zeros((n_steps, n_states))
        u_traj = np.zeros((n_steps, n_inputs))
        
        # Initial condition
        x = x0.copy()
        x_traj[0] = x
        
        # Closed-loop system matrix
        A_cl = self.A - self.B @ self.K
        
        # Simulation loop
        for i in range(1, n_steps):
            # Control law
            error = x - xf
            u = -self.K @ error
            u_traj[i-1] = u
            
            # State update (Euler integration of error dynamics)
            error_dot = A_cl @ error
            x = x + (self.A @ x + self.B @ u) * dt  # Full dynamics
            x_traj[i] = x
        
        # Final control input
        error = x_traj[-1] - xf
        u_traj[-1] = -self.K @ error
        
        return t, x_traj, u_traj

    def compute_lqr_cost(self, x_traj, u_traj, xf, dt):
        """Compute LQR cost along trajectory"""
        cost = 0
        for i in range(len(x_traj)):
            x_error = x_traj[i] - xf
            cost += (x_error.T @ self.Q @ x_error + u_traj[i].T @ self.R @ u_traj[i]) * dt
        return cost

    def test_step_response(self, plot_results=True):
        """Test step response and plot results"""
        print("\n" + "="*60)
        print("STEP RESPONSE TEST")
        print("="*60)
        
        # Test cases
        test_cases = [
            {"name": "5m step in X", "x0": np.zeros(12), "xf": np.array([5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])},
            {"name": "2m step in Z", "x0": np.zeros(12), "xf": np.array([0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0])},
            {"name": "0.1 rad roll", "x0": np.zeros(12), "xf": np.array([0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0])}
        ]
        
        if plot_results:
            fig, axes = plt.subplots(len(test_cases), 2, figsize=(15, 5*len(test_cases)))
            if len(test_cases) == 1:
                axes = axes.reshape(1, -1)
        
        for idx, test_case in enumerate(test_cases):
            print(f"\nTest: {test_case['name']}")
            
            t, x_traj, u_traj = self.simulate_step_response(
                test_case["x0"], test_case["xf"], t_final=8
            )
            
            if t is None:
                continue
                
            # Compute cost
            cost = self.compute_lqr_cost(x_traj, u_traj, test_case["xf"], t[1]-t[0])
            print(f"  LQR cost: {cost:.3f}")
            
            # Check settling time (2% criterion)
            error_norm = np.linalg.norm(x_traj - test_case["xf"], axis=1)
            final_error = error_norm[-1]
            settling_indices = np.where(error_norm <= 0.02 * np.max(error_norm))[0]
            if len(settling_indices) > 0:
                settling_time = t[settling_indices[0]]
                print(f"  Settling time (2%): {settling_time:.2f}s")
            
            # Max control effort
            max_control = np.max(np.abs(u_traj))
            print(f"  Max control effort: {max_control:.3f}")
            
            if plot_results:
                # Plot states
                ax1 = axes[idx, 0]
                for i, state_name in enumerate(self.state_names[:6]):  # Plot first 6 states
                    if np.max(np.abs(x_traj[:, i])) > 1e-6:  # Only plot non-zero states
                        ax1.plot(t, x_traj[:, i], label=f'{state_name}')
                
                # Plot target
                target_states = np.where(np.abs(test_case["xf"]) > 1e-6)[0]
                for i in target_states:
                    if i < 6:
                        ax1.axhline(y=test_case["xf"][i], color='r', linestyle='--', alpha=0.7)
                
                ax1.set_ylabel('States')
                ax1.set_xlabel('Time (s)')
                ax1.set_title(f'{test_case["name"]} - States')
                ax1.legend()
                ax1.grid(True, alpha=0.3)
                
                # Plot control inputs
                ax2 = axes[idx, 1]
                for i, input_name in enumerate(self.input_names):
                    ax2.plot(t, u_traj[:, i], label=f'{input_name}')
                ax2.set_ylabel('Control Input')
                ax2.set_xlabel('Time (s)')
                ax2.set_title(f'{test_case["name"]} - Control')
                ax2.legend()
                ax2.grid(True, alpha=0.3)
        
        if plot_results:
            plt.tight_layout()
            plt.show(block=False)

    def run_all_tests(self):
        """Run comprehensive LQR verification"""
        print("="*80)
        print("LQR CONTROLLER COMPREHENSIVE VERIFICATION")
        print("="*80)
        
        # 1. Controllability
        print("\n1. CONTROLLABILITY CHECK")
        print("-" * 40)
        self.check_controllability()
        
        # 2. Solve LQR
        print("\n2. SOLVING LQR")
        print("-" * 40)
        print("Solving Algebraic Riccati Equation...")
        self.solve_lqr()
        print("LQR solution completed.")
        
        # 3. ARE verification
        print("\n3. ARE SOLUTION VERIFICATION")
        print("-" * 40)
        self.verify_are_solution()
        
        # 4. Stability
        print("\n4. CLOSED-LOOP STABILITY CHECK")
        print("-" * 40)
        is_stable, eigenvalues_cl = self.check_closed_loop_stability()
        
        # 5. Gain analysis
        print("\n5. GAIN MATRIX ANALYSIS")
        print("-" * 40)
        self.analyze_gains()
        
        # 6. Step response tests
        print("\n6. STEP RESPONSE TESTS")
        print("-" * 40)
        self.test_step_response()
        
        # 7. Summary
        print("\n" + "="*80)
        print("VERIFICATION SUMMARY")
        print("="*80)
        print(f"✓ Controllability: {'PASS' if self.check_controllability() else 'FAIL'}")
        print(f"✓ ARE Solution:    {'PASS' if self.verify_are_solution() else 'FAIL'}")
        print(f"✓ Stability:       {'PASS' if is_stable else 'FAIL'}")
        print(f"✓ Gain Matrix:     Available ({self.K.shape[0]}x{self.K.shape[1]})")
        print("✓ Step Response:   Completed")
        
        return {
            'controllable': self.check_controllability(),
            'are_valid': self.verify_are_solution(),
            'stable': is_stable,
            'K': self.K,
            'P': self.P,
            'eigenvalues_cl': eigenvalues_cl
        }

def plot_eigenvalues(eigenvalues_A, eigenvalues_A_cl):
    """Plot eigenvalues of A and A_cl in the complex plane."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    # Plot eigenvalues of A (open-loop)
    real_A = np.real(eigenvalues_A)
    imag_A = np.imag(eigenvalues_A)
    ax1.scatter(real_A, imag_A, c='red', s=100, marker='x', linewidth=3, label='Open-loop (A)')
    ax1.axvline(x=0, color='black', linestyle='--', alpha=0.3)
    ax1.axhline(y=0, color='black', linestyle='--', alpha=0.3)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('Real Part')
    ax1.set_ylabel('Imaginary Part')
    ax1.set_title('Eigenvalues of A (Open-loop System)')
    ax1.legend()
    
    # Plot eigenvalues of A_cl (closed-loop)
    real_A_cl = np.real(eigenvalues_A_cl)
    imag_A_cl = np.imag(eigenvalues_A_cl)
    ax2.scatter(real_A_cl, imag_A_cl, c='blue', s=100, marker='o', label='Closed-loop (A_cl)')
    ax2.axvline(x=0, color='black', linestyle='--', alpha=0.3)
    ax2.axhline(y=0, color='black', linestyle='--', alpha=0.3)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('Real Part')
    ax2.set_ylabel('Imaginary Part')
    ax2.set_title('Eigenvalues of A_cl (Closed-loop System)')
    ax2.legend()
    
    plt.tight_layout()
    plt.show(block=False)
    
    # Combined plot
    plt.figure(figsize=(8, 6))
    plt.scatter(real_A, imag_A, c='red', s=100, marker='x', linewidth=3, label='Open-loop (A)')
    plt.scatter(real_A_cl, imag_A_cl, c='blue', s=100, marker='o', label='Closed-loop (A_cl)')
    plt.axvline(x=0, color='black', linestyle='--', alpha=0.3)
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.3)
    plt.grid(True, alpha=0.3)
    plt.xlabel('Real Part')
    plt.ylabel('Imaginary Part')
    plt.title('Eigenvalues Comparison: Open-loop vs Closed-loop')
    plt.legend()
    plt.tight_layout()
    plt.show(block=False)

def main():
    # Create controller
    lqr_controller = LQRController()
    
    # Run comprehensive tests
    results = lqr_controller.run_all_tests()
    
    # Original simple test
    print("\n" + "="*60)
    print("ORIGINAL SIMPLE TEST")
    print("="*60)
    
    K = lqr_controller.get_K()
    print("Gain Matrix K:")
    print(K)
    
    # Eigenvalue analysis
    eigenvalues_A = lqr_controller.get_eigenvalues()
    A_cl = lqr_controller.A - lqr_controller.B @ K
    eigenvalues_A_cl = np.linalg.eigvals(A_cl)
    
    print(f"\nEigenvalues of A: {eigenvalues_A}")
    print(f"Eigenvalues of A_cl: {eigenvalues_A_cl}")
    
    # Plot eigenvalues
    plot_eigenvalues(eigenvalues_A, eigenvalues_A_cl)
    
    # Simple control test
    X_i = np.array([1.67, 0.205, 0.009, 1.97, 0.015, 0.0240, 2.191e-3, -5.1065e-4, -1.4297e-4, 0.00372, 0.01574, -0.06477])  # Initial state
    X_f = np.array([4.45, 0.212, 0.010, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # Final state
    X = X_i - X_f  # State error
    U = -K @ X  # Control input
    print(f"\nControl Input U for X is: {U}")

    # print("Press Enter to exit...")
    # input()
    # plt.close('all')  # Close all plots

if __name__ == "__main__":
    main()