#!/usr/bin/env python3
"""
Visualize the calibration results by plotting original and corrected trajectories.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def load_trajectory(filename):
    """Load trajectory data from file."""
    data = np.loadtxt(filename, comments='#')
    return data

def main():
    # Default file paths
    wheel_file = "/home/jay/catkin_lw/wheel_odom.txt"
    lidar_file = "/home/jay/catkin_lw/lidar_odom.txt"
    corrected_file = "corrected_lidar_path.txt"
    
    # Check if corrected file exists
    if not os.path.exists(corrected_file):
        print(f"Error: Cannot find {corrected_file}")
        print("Please run the calibration first.")
        return
    
    # Load trajectories
    print("Loading trajectories...")
    wheel_data = load_trajectory(wheel_file)
    lidar_data = load_trajectory(lidar_file)
    corrected_data = load_trajectory(corrected_file)
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot original trajectories
    ax1.plot(wheel_data[:, 1], wheel_data[:, 2], 'b-', 
             label='Wheel Odometry', linewidth=2)
    ax1.plot(lidar_data[:, 1], lidar_data[:, 2], 'r-', 
             label='LiDAR Odometry', linewidth=2)
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('Original Trajectories', fontsize=14)
    ax1.legend(fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # Plot calibrated trajectories
    ax2.plot(wheel_data[:, 1], wheel_data[:, 2], 'b-', 
             label='Wheel Odometry', linewidth=2)
    ax2.plot(corrected_data[:, 1], corrected_data[:, 2], 'g-', 
             label='Calibrated LiDAR', linewidth=2, alpha=0.8)
    ax2.set_xlabel('X (m)', fontsize=12)
    ax2.set_ylabel('Y (m)', fontsize=12)
    ax2.set_title('Calibrated Trajectories (After Extrinsic Correction)', fontsize=14)
    ax2.legend(fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    plt.tight_layout()
    plt.savefig('cpp_calibration_result.png', dpi=150, bbox_inches='tight')
    print("Saved visualization to cpp_calibration_result.png")
    
    # Calculate alignment error
    position_errors = np.linalg.norm(
        wheel_data[:, 1:3] - corrected_data[:, 1:3], 
        axis=1
    )
    
    print("\nAlignment Quality Metrics:")
    print(f"  Mean position error:   {np.mean(position_errors):.6f} m")
    print(f"  Std position error:    {np.std(position_errors):.6f} m")
    print(f"  Max position error:    {np.max(position_errors):.6f} m")
    print(f"  Median position error: {np.median(position_errors):.6f} m")
    
    plt.show()

if __name__ == "__main__":
    main()