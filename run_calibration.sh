#!/bin/bash
# Script to run the Ceres calibration with the correct data files

# Check if we're in the build directory or the project root
if [ -f "extrinsic_calibrator" ]; then
    # We're in the build directory
    EXEC="./extrinsic_calibrator"
elif [ -f "build/extrinsic_calibrator" ]; then
    # We're in the project root
    EXEC="./build/extrinsic_calibrator"
else
    echo "Error: Cannot find extrinsic_calibrator executable"
    echo "Please build the project first:"
    echo "  mkdir build && cd build"
    echo "  cmake .."
    echo "  make"
    exit 1
fi

# Path to data files
WHEEL_ODOM="/home/jay/catkin_lw/wheel_odom.txt"
LIDAR_ODOM="/home/jay/catkin_lw/lidar_odom.txt"

# Check if data files exist
if [ ! -f "$WHEEL_ODOM" ]; then
    echo "Error: Cannot find wheel_odom.txt at $WHEEL_ODOM"
    exit 1
fi

if [ ! -f "$LIDAR_ODOM" ]; then
    echo "Error: Cannot find lidar_odom.txt at $LIDAR_ODOM"
    exit 1
fi

# Run the calibration
echo "Running calibration with:"
echo "  Wheel odometry: $WHEEL_ODOM"
echo "  LiDAR odometry: $LIDAR_ODOM"
echo ""

$EXEC "$WHEEL_ODOM" "$LIDAR_ODOM"