import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

G = 9.81
DIST_COM = 0.275170
I_XX = 0.00075540
I_YY = 0.00440442
I_ZZ = 0.00458442
MASS = 0.562    

# X_i = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]) # Initial state
# X_f = np.array([2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]) # Final state

class LQRController:
    def __init__(self, q_config='default', r_config='default'):
        """
        Initializes the LQR controller with system matrices A, B and cost matrices Q, R.
        
        Args:
            q_config (str): Configuration for Q matrix ('default', 'stable', 'very_stable', etc.)
            r_config (str): Configuration for R matrix ('default', 'aggressive', 'conservative', etc.)
        """
        self.A = self.set_A()
        assert self.A.shape[0] == self.A.shape[1], "Matrix A must be square."
        assert self.A.shape[0] == 12, "Matrix A must be of size 12x12."

        self.B = self.set_B()
        assert self.B.shape == (12, 4), "Matrix B must be of size 12x4."

        self.Q = self.set_Q(q_config)
        self.R = self.set_R(r_config)
        self.P = None
        self.K = None
        self.q_config = q_config
        self.r_config = r_config

    def solve_lqr(self):
        """Solves the LQR problem and returns the gain matrix K and solution P."""
        self.P = scipy.linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        # self.K = np.linalg.inv(self.R + self.B.T @ self.P @ self.B) @\
        #     (self.B.T @ self.P @ self.A)

        self.K = np.linalg.solve(self.R, self.B.T @ self.P)

    def get_K(self):
        return self.K
    
    def get_P(self):
        return self.P

    def set_A(self):
        """
        Constructs the state transition matrix A for the system dynamics.
        Returns:
            A (np.ndarray): The state transition matrix (n x n).
        """
        # return np.array([
        #     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # x
        #     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],    # y
        #     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],    # z
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # x_dot
        #     [0, 0, 0, 0, 0, 0, 0, 0, -2*G, 0, 0, 0, 0],  # y_dot
        #     [0, 0, 0, 0, 0, 0, 0, -2*G, 0, 0, 0, 0, 0],    # z_dot
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0, 0],  # q_x
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0],  # q_y
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2],  # q_z
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # q_w
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # p
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],    # q
        #     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]     # r
        # ])
        return np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, -2*G, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, -2*G, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/2],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ])
    
    def set_B(self):
        # return np.array([
        #     [0, 0, 0],  # x
        #     [0, 0, 0],  # y
        #     [0, 0, 0],  # z
        #     [1/MASS, 0, 0],  # x_dot
        #     [0, 1/MASS, 0],  # y_dot
        #     [0, 0, 1/MASS],  # z_dot
        #     [0, 0, 0],  # q_x
        #     [0, 0, 0],  # q_y
        #     [0, 0, 0],  # q_z
        #     [0, 0, 0],  # q_w
        #     [0, 0, 0],  # p
        #     [0, 0, DIST_COM/I_YY],  # q
        #     [0, -DIST_COM/I_ZZ, 0]  # r
        # ])
        return np.array([ 
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1/MASS, 0, 0, 0],
            [0, 1/MASS, 0, 0],
            [0, 0, 1/MASS, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1/I_XX],
            [0, 0, DIST_COM/I_YY, 0],
            [0, -DIST_COM/I_ZZ, 0, 0]
        ])
    
    def set_Q(self, config='default'):
        """Set Q matrix with different stability configurations."""
        configs = {
            'default': [4.5, 1, 1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01],
            'stable': [10, 5, 5, 1, 1, 1, 1, 1, 1, 0.1, 0.1, 0.1],
            'very_stable': [50, 20, 20, 5, 5, 5, 5, 5, 5, 1, 1, 1],
            'position_focused': [100, 10, 10, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01],
            'velocity_focused': [4.5, 1, 1, 10, 10, 10, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01],
            'attitude_focused': [4.5, 1, 1, 0.1, 0.1, 0.1, 10, 10, 10, 1, 1, 1]
        }
        return np.diag(configs[config])
    
    def set_R(self, config='default'):
        """Set R matrix with different control effort configurations."""
        configs = {
            'default': [1, 10, 10, 1],
            'aggressive': [0.1, 1, 1, 0.1],
            'conservative': [10, 50, 50, 10],
            'balanced': [1, 5, 5, 1]
        }
        return np.diag(configs[config])
    
    def get_eigenvalues(self):
        """Returns the eigenvalues of the system matrix A."""
        return np.linalg.eigvals(self.A)

def compare_stability_configs():
    """Compare different Q/R configurations and their stability."""
    configs = [
        ('default', 'default'),
        ('stable', 'default'),
        ('very_stable', 'aggressive'),
        ('position_focused', 'aggressive'),
        ('velocity_focused', 'balanced'),
        ('attitude_focused', 'balanced')
    ]
    
    results = []
    
    for q_config, r_config in configs:
        controller = LQRController(q_config, r_config)
        controller.solve_lqr()
        
        A_cl = controller.A - controller.B @ controller.K
        eigenvalues = np.linalg.eigvals(A_cl)
        max_real = np.max(np.real(eigenvalues))
        
        results.append({
            'config': f"Q:{q_config}, R:{r_config}",
            'max_real': max_real,
            'stable': max_real < -0.01,  # Small margin for numerical stability
            'eigenvalues': eigenvalues
        })
    
    # Print comparison table
    print("Stability Comparison:")
    print("-" * 60)
    print(f"{'Configuration':<35} {'Max Real Part':<15} {'Stable'}")
    print("-" * 60)
    
    for result in results:
        stable_str = "✓" if result['stable'] else "✗"
        print(f"{result['config']:<35} {result['max_real']:<15.6f} {stable_str}")
    
    return results

def plot_stability_comparison(results):
    """Plot eigenvalues for different configurations."""
    n_configs = len(results)
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown']
    
    for i, result in enumerate(results):
        eigenvalues = result['eigenvalues']
        real_parts = np.real(eigenvalues)
        imag_parts = np.imag(eigenvalues)
        
        axes[i].scatter(real_parts, imag_parts, c=colors[i], s=100, marker='o')
        axes[i].axvline(x=0, color='black', linestyle='--', alpha=0.3)
        axes[i].axhline(y=0, color='black', linestyle='--', alpha=0.3)
        axes[i].grid(True, alpha=0.3)
        axes[i].set_xlabel('Real Part')
        axes[i].set_ylabel('Imaginary Part')
        axes[i].set_title(f"{result['config']}\nMax Real: {result['max_real']:.3f}")
        
        # Highlight stability region
        if result['stable']:
            axes[i].axvspan(-10, 0, alpha=0.1, color='green', label='Stable region')
        else:
            axes[i].axvspan(0, 10, alpha=0.1, color='red', label='Unstable region')
    
    plt.tight_layout()
    plt.show()

def main():
    # Compare different stability configurations
    print("=== STABILITY ANALYSIS ===")
    results = compare_stability_configs()
    plot_stability_comparison(results)
    
    print("\n=== DETAILED ANALYSIS FOR BEST CONFIGURATION ===")
    # Use the most stable configuration
    lqr_controller = LQRController('stable', 'default')
    lqr_controller.solve_lqr()
    
    K = lqr_controller.get_K()
    P = lqr_controller.get_P()
    
    print("Gain Matrix K:")
    print(K)

    A = lqr_controller.A
    B = lqr_controller.B
    A_cl = A - B @ K

    print("\nEigenvalues of A (open-loop):")
    eigenvalues = lqr_controller.get_eigenvalues()
    print(eigenvalues)  

    print("\nEigenvalues of A_cl (closed-loop):")
    eigenvalues_cl = np.linalg.eigvals(A_cl)
    print(eigenvalues_cl)

    # Check stability
    max_real_part = np.max(np.real(eigenvalues_cl))
    print(f"\nMaximum real part of A_cl eigenvalues: {max_real_part:.6f}")
    if max_real_part < -0.01:
        print("System is stable (all eigenvalues have sufficiently negative real parts)")
    elif max_real_part < 0:
        print("System is marginally stable (eigenvalues very close to imaginary axis)")
    else:
        print("System is unstable (some eigenvalues have non-negative real parts)")

    # Test case with improved controller
    X_i = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) # Initial state
    X_f = np.array([5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]) # Final state
    X = X_i - X_f  # State error
    U = - K @ X  # Control input
    print("\nControl Input U for 5m displacement:")
    print(U)
    
    # Show control effort comparison
    original_controller = LQRController('default', 'default')
    original_controller.solve_lqr()
    U_original = - original_controller.get_K() @ X
    
    print("\nControl effort comparison:")
    print(f"Original config control effort: {np.linalg.norm(U_original):.4f}")
    print(f"Stable config control effort:   {np.linalg.norm(U):.4f}")
    print(f"Ratio (stable/original):        {np.linalg.norm(U)/np.linalg.norm(U_original):.4f}")

if __name__ == "__main__":
    main()