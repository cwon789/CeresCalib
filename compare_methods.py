#!/usr/bin/env python3
"""
Compare the two different methods of generating the corrected path:
1. Simple transformation of the whole path (incorrect)
2. Applying extrinsic to each incremental movement (correct)
"""

import numpy as np
import matplotlib.pyplot as plt

def load_trajectory(filename):
    """Load trajectory data from file."""
    data = np.loadtxt(filename, comments='#')
    return data

def simple_transform_path(lidar_poses, dx, dy, dtheta):
    """
    Simple (incorrect) method: Just transform the entire path
    """
    c = np.cos(dtheta)
    s = np.sin(dtheta)
    
    corrected = []
    for pose in lidar_poses:
        # Simple rotation and translation
        x_new = c * pose[1] - s * pose[2] + dx
        y_new = s * pose[1] + c * pose[2] + dy
        theta_new = pose[3] + dtheta
        corrected.append([pose[0], x_new, y_new, theta_new])
    
    return np.array(corrected)

def main():
    # Load data
    print("Loading trajectories...")
    wheel_data = load_trajectory("/home/jay/catkin_lw/wheel_odom.txt")
    lidar_data = load_trajectory("/home/jay/catkin_lw/lidar_odom.txt")
    corrected_data = load_trajectory("corrected_lidar_path.txt")
    
    # Extrinsic parameters from optimization
    dx = 0.363789
    dy = -0.255110
    dtheta = -0.749364
    
    # Generate simple transform path for comparison
    simple_corrected = simple_transform_path(lidar_data, dx, dy, dtheta)
    
    # Create figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20, 6))
    
    # Plot 1: Original trajectories
    ax1.plot(wheel_data[:, 1], wheel_data[:, 2], 'b-', 
             label='Wheel Odometry', linewidth=2)
    ax1.plot(lidar_data[:, 1], lidar_data[:, 2], 'r-', 
             label='LiDAR Odometry', linewidth=2)
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('Original Trajectories', fontsize=14)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot 2: Simple transform (incorrect)
    ax2.plot(wheel_data[:, 1], wheel_data[:, 2], 'b-', 
             label='Wheel Odometry', linewidth=2)
    ax2.plot(simple_corrected[:, 1], simple_corrected[:, 2], 'r--', 
             label='Simple Transform (Incorrect)', linewidth=2, alpha=0.8)
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Y (m)', fontsize=12)
    ax2.set_title('Simple Transformation Method', fontsize=14)
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # Plot 3: Incremental transform (correct)
    ax3.plot(wheel_data[:, 1], wheel_data[:, 2], 'b-', 
             label='Wheel Odometry', linewidth=2)
    ax3.plot(corrected_data[:, 1], corrected_data[:, 2], 'g-', 
             label='Incremental Transform (Correct)', linewidth=2, alpha=0.8)
    ax3.set_xlabel('X (m)', fontsize=12)
    ax3.set_ylabel('Y (m)', fontsize=12)
    ax3.set_title('Incremental Transformation Method', fontsize=14)
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    plt.suptitle('Comparison of Path Correction Methods', fontsize=16)
    plt.tight_layout()
    plt.savefig('correction_method_comparison.png', dpi=150, bbox_inches='tight')
    print("Saved comparison to correction_method_comparison.png")
    
    # Calculate errors for both methods
    simple_errors = np.linalg.norm(
        wheel_data[:, 1:3] - simple_corrected[:, 1:3], 
        axis=1
    )
    
    incremental_errors = np.linalg.norm(
        wheel_data[:, 1:3] - corrected_data[:, 1:3], 
        axis=1
    )
    
    print("\nError Statistics:")
    print("-" * 50)
    print("Simple Transform Method (Incorrect):")
    print(f"  Mean error:   {np.mean(simple_errors):.6f} m")
    print(f"  Max error:    {np.max(simple_errors):.6f} m")
    print(f"  Final error:  {simple_errors[-1]:.6f} m")
    
    print("\nIncremental Transform Method (Correct):")
    print(f"  Mean error:   {np.mean(incremental_errors):.6f} m")
    print(f"  Max error:    {np.max(incremental_errors):.6f} m")
    print(f"  Final error:  {incremental_errors[-1]:.6f} m")
    
    plt.show()

if __name__ == "__main__":
    main()