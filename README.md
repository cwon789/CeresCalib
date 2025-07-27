# LiDAR Extrinsic Calibration with Ceres Solver

This project implements high-precision 3-DOF extrinsic calibration between wheel odometry and LiDAR odometry using the Ceres Solver library.

## Critical Implementation Detail: Path Correction Method

### The Problem

When verifying extrinsic calibration results, we need to generate a "corrected" LiDAR path that should align with the wheel odometry. There are two approaches:

1. **Simple Transform (INCORRECT)**: Apply the extrinsic transformation to the entire LiDAR trajectory
2. **Incremental Transform (CORRECT)**: Apply the extrinsic to each incremental movement

### Why Simple Transform is Wrong

The simple approach of just transforming the entire path:
```cpp
T_corrected = X * T_lidar_original
```

This is incorrect because it only changes the reference frame of the trajectory without accounting for how the extrinsic affects the accumulation of errors over time.

### The Correct Approach: Incremental Transform

The correct method applies the extrinsic transformation to each relative motion:

```cpp
for each pose i:
    T_lidar_relative = inv(T_lidar_i) * T_lidar_{i+1}
    T_base_equivalent = X * T_lidar_relative * inv(X)
    T_corrected = T_corrected * T_base_equivalent
```

This properly accounts for how the extrinsic calibration affects the shape of the trajectory.

## Building the Project

```bash
mkdir build
cd build
cmake ..
make
```

## Running the Calibration

```bash
# Run the corrected version
./extrinsic_calibrator_corrected wheel_odom.txt lidar_odom.txt

# Compare with the simple version
./extrinsic_calibrator wheel_odom.txt lidar_odom.txt
```

## Implementation Details

### Key Functions

1. **`generateCorrectedPath()`**: Implements the correct incremental transformation method
   - Starts with identity transformation at origin
   - For each pose, calculates relative motion in LiDAR frame
   - Applies kinematic transformation: `T_base = X * T_lidar_rel * inv(X)`
   - Chains transformations to build corrected path

2. **`ExtrinsicCalibrationCostFunctor`**: Ceres cost function
   - Computes kinematic error: `Error = inv(T_wheel) * X * T_lidar * inv(X)`
   - Returns 3D residual vector (x, y, theta errors)

3. **Robust Optimization**: Uses Huber loss function
   - Scale parameter: 0.1
   - Provides robustness against outliers in odometry data

### Output Files

- `corrected_lidar_path.txt`: The corrected LiDAR trajectory using the proper incremental method
- Console output: Detailed optimization statistics and final extrinsic parameters

## Visualization

Use the provided Python scripts to visualize results:
```bash
python3 compare_methods.py      # Compare correction methods
python3 visualize_results.py    # Basic visualization
```

## Mathematical Background

The calibration finds the transformation X that minimizes:
```
sum_i || inv(T_wheel_i) * X * T_lidar_i * inv(X) - I ||²
```

Where:
- `T_wheel_i`: Relative transformation from wheel odometry
- `T_lidar_i`: Relative transformation from LiDAR odometry
- `X`: The extrinsic transformation (what we're solving for)
- `I`: Identity matrix (ideal error-free case)