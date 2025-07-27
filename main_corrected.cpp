/**
 * High-precision offline 3-DOF extrinsic calibration between wheel and LiDAR odometry.
 * This program uses the Ceres Solver to find optimal extrinsic parameters
 * (dx, dy, d_theta) that minimize kinematic error between two odometry sources.
 * 
 * CRITICAL: The corrected path generation properly applies the extrinsic to each
 * incremental movement, not just a simple transformation of the whole path.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <chrono>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Data structure for a pose
struct Pose {
    double timestamp;
    double x;
    double y;
    double theta;
    
    Pose() : timestamp(0.0), x(0.0), y(0.0), theta(0.0) {}
    Pose(double t, double x_, double y_, double theta_) 
        : timestamp(t), x(x_), y(y_), theta(theta_) {}
};

// Data structure for a 3x3 transformation matrix
struct Transform {
    Eigen::Matrix3d matrix;
    
    Transform() : matrix(Eigen::Matrix3d::Identity()) {}
    
    Transform(double x, double y, double theta) {
        double c = std::cos(theta);
        double s = std::sin(theta);
        matrix << c, -s, x,
                  s,  c, y,
                  0,  0, 1;
    }
    
    Transform inverse() const {
        Transform inv;
        inv.matrix = matrix.inverse();
        return inv;
    }
    
    Transform operator*(const Transform& other) const {
        Transform result;
        result.matrix = matrix * other.matrix;
        return result;
    }
};

// Normalize angle to [-pi, pi]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Load odometry data from file
std::vector<Pose> loadOdometryData(const std::string& filename) {
    std::vector<Pose> poses;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        std::cerr << "Please ensure the file exists and you have read permissions." << std::endl;
        exit(1);
    }
    
    std::string line;
    double timestamp, x, y, theta;
    
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream iss(line);
        if (iss >> timestamp >> x >> y >> theta) {
            poses.emplace_back(timestamp, x, y, theta);
        }
    }
    
    file.close();
    
    if (poses.empty()) {
        std::cerr << "Error: No data loaded from " << filename << std::endl;
        std::cerr << "Please check the file format. Expected format: timestamp x y theta" << std::endl;
        exit(1);
    }
    
    std::cout << "Loaded " << poses.size() << " poses from " << filename << std::endl;
    return poses;
}

// Interpolate angle considering wrapping
double interpolateAngle(double theta1, double theta2, double alpha) {
    // Compute difference considering wrapping
    double diff = normalizeAngle(theta2 - theta1);
    return normalizeAngle(theta1 + alpha * diff);
}

// Interpolate odometry to target timestamps
std::vector<Pose> interpolateOdometry(const std::vector<Pose>& targetTimestamps,
                                     const std::vector<Pose>& sourcePoses) {
    std::vector<Pose> interpolated;
    interpolated.reserve(targetTimestamps.size());
    
    for (const auto& target : targetTimestamps) {
        double targetTime = target.timestamp;
        
        // Find the two source poses to interpolate between
        auto it = std::lower_bound(sourcePoses.begin(), sourcePoses.end(), targetTime,
                                  [](const Pose& p, double t) { return p.timestamp < t; });
        
        if (it == sourcePoses.begin()) {
            // Target time is before all source times
            interpolated.push_back(sourcePoses.front());
        } else if (it == sourcePoses.end()) {
            // Target time is after all source times
            interpolated.push_back(sourcePoses.back());
        } else {
            // Normal interpolation case
            const Pose& p2 = *it;
            const Pose& p1 = *(it - 1);
            
            // Calculate interpolation factor
            double alpha = (targetTime - p1.timestamp) / (p2.timestamp - p1.timestamp);
            
            // Linearly interpolate x and y
            double x = p1.x + alpha * (p2.x - p1.x);
            double y = p1.y + alpha * (p2.y - p1.y);
            
            // Interpolate angle with wrapping
            double theta = interpolateAngle(p1.theta, p2.theta, alpha);
            
            interpolated.emplace_back(targetTime, x, y, theta);
        }
    }
    
    return interpolated;
}

// Convert pose to transformation matrix
Transform poseToTransform(const Pose& pose) {
    return Transform(pose.x, pose.y, pose.theta);
}

// Convert transformation matrix to pose
Pose transformToPose(const Transform& transform) {
    Pose pose;
    pose.x = transform.matrix(0, 2);
    pose.y = transform.matrix(1, 2);
    pose.theta = std::atan2(transform.matrix(1, 0), transform.matrix(0, 0));
    return pose;
}

// Compute relative transformations between consecutive poses
std::vector<Transform> computeRelativeTransforms(const std::vector<Pose>& poses) {
    std::vector<Transform> relativeTransforms;
    
    if (poses.size() < 2) {
        std::cerr << "Error: Need at least 2 poses to compute relative transforms" << std::endl;
        return relativeTransforms;
    }
    
    relativeTransforms.reserve(poses.size() - 1);
    
    for (size_t i = 1; i < poses.size(); ++i) {
        Transform T_prev = poseToTransform(poses[i - 1]);
        Transform T_curr = poseToTransform(poses[i]);
        Transform T_relative = T_prev.inverse() * T_curr;
        relativeTransforms.push_back(T_relative);
    }
    
    return relativeTransforms;
}

// Ceres cost function functor for extrinsic calibration
struct ExtrinsicCalibrationCostFunctor {
    const Transform T_wheel;
    const Transform T_lidar;
    
    ExtrinsicCalibrationCostFunctor(const Transform& wheel, const Transform& lidar)
        : T_wheel(wheel), T_lidar(lidar) {}
    
    template <typename T>
    bool operator()(const T* const extrinsic, T* residual) const {
        // Extract extrinsic parameters
        T dx = extrinsic[0];
        T dy = extrinsic[1];
        T d_theta = extrinsic[2];
        
        // Create extrinsic transformation matrix
        T c = ceres::cos(d_theta);
        T s = ceres::sin(d_theta);
        
        Eigen::Matrix<T, 3, 3> X_guess;
        X_guess << c, -s, dx,
                   s,  c, dy,
                   T(0), T(0), T(1);
        
        // Convert wheel and lidar transforms to Eigen matrices with type T
        Eigen::Matrix<T, 3, 3> T_wheel_eigen = T_wheel.matrix.cast<T>();
        Eigen::Matrix<T, 3, 3> T_lidar_eigen = T_lidar.matrix.cast<T>();
        
        // Compute kinematic error: Error = inv(T_wheel) * X * T_lidar * inv(X)
        Eigen::Matrix<T, 3, 3> X_guess_inv = X_guess.inverse();
        Eigen::Matrix<T, 3, 3> T_wheel_inv = T_wheel_eigen.inverse();
        Eigen::Matrix<T, 3, 3> Error = T_wheel_inv * X_guess * T_lidar_eigen * X_guess_inv;
        
        // Extract error components
        residual[0] = Error(0, 2);  // error_x
        residual[1] = Error(1, 2);  // error_y
        residual[2] = ceres::atan2(Error(1, 0), Error(0, 0));  // error_theta
        
        return true;
    }
};

// Generate corrected path by applying extrinsic to each incremental movement
std::vector<Pose> generateCorrectedPath(const std::vector<Pose>& lidarPoses,
                                       const double* extrinsic) {
    std::vector<Pose> correctedPath;
    correctedPath.reserve(lidarPoses.size());
    
    if (lidarPoses.empty()) {
        return correctedPath;
    }
    
    // Create extrinsic transformation matrix
    Transform X(extrinsic[0], extrinsic[1], extrinsic[2]);
    Transform X_inv = X.inverse();
    
    // Start with identity transformation (origin)
    Transform T_corrected;  // Identity by default
    
    // Add the first pose at origin
    Pose firstPose(lidarPoses[0].timestamp, 0.0, 0.0, 0.0);
    correctedPath.push_back(firstPose);
    
    // Process each incremental movement
    for (size_t i = 0; i < lidarPoses.size() - 1; ++i) {
        // Calculate relative motion in LiDAR frame
        Transform T_lidar_i = poseToTransform(lidarPoses[i]);
        Transform T_lidar_i_plus_1 = poseToTransform(lidarPoses[i + 1]);
        Transform T_lidar_relative = T_lidar_i.inverse() * T_lidar_i_plus_1;
        
        // Apply kinematic transformation to find equivalent motion in base_link frame
        // T_base_equivalent = X * T_lidar_relative * inv(X)
        Transform T_base_equivalent = X * T_lidar_relative * X_inv;
        
        // Update corrected path by chaining this transform
        T_corrected = T_corrected * T_base_equivalent;
        
        // Extract pose from the corrected transformation
        Pose correctedPose = transformToPose(T_corrected);
        correctedPose.timestamp = lidarPoses[i + 1].timestamp;
        
        correctedPath.push_back(correctedPose);
    }
    
    return correctedPath;
}

// Save corrected trajectory to file
void saveCorrectedTrajectory(const std::vector<Pose>& correctedPoses,
                           const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create output file " << filename << std::endl;
        return;
    }
    
    // Write header
    file << "# Corrected LiDAR trajectory after extrinsic calibration" << std::endl;
    file << "# timestamp x y theta_in_radians" << std::endl;
    
    // Write poses
    file << std::fixed << std::setprecision(6);
    for (const auto& pose : correctedPoses) {
        file << pose.timestamp << " " 
             << pose.x << " " 
             << pose.y << " " 
             << pose.theta << std::endl;
    }
    
    file.close();
    std::cout << "Saved corrected trajectory to " << filename << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "======================================================================" << std::endl;
    std::cout << "High-Precision Offline 3-DOF Extrinsic Calibration" << std::endl;
    std::cout << "Wheel Odometry <-> LiDAR Odometry" << std::endl;
    std::cout << "Using Ceres Solver with Robust Loss Function" << std::endl;
    std::cout << "======================================================================" << std::endl;
    
    // Parse command line arguments
    std::string wheel_file = "/home/jay/catkin_lw/wheel_odom.txt";
    std::string lidar_file = "/home/jay/catkin_lw/lidar_odom.txt";
    
    if (argc >= 3) {
        wheel_file = argv[1];
        lidar_file = argv[2];
    } else if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [wheel_odom.txt] [lidar_odom.txt]" << std::endl;
        std::cout << "If no arguments provided, looks for files in current directory." << std::endl;
        return 0;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Step 1: Load odometry data
    std::cout << "\n[1/6] Loading odometry data..." << std::endl;
    std::vector<Pose> wheelPoses = loadOdometryData(wheel_file);
    std::vector<Pose> lidarPoses = loadOdometryData(lidar_file);
    
    // Step 2: Interpolate wheel odometry to LiDAR timestamps
    std::cout << "\n[2/6] Interpolating wheel odometry to LiDAR timestamps..." << std::endl;
    std::vector<Pose> wheelPosesInterp = interpolateOdometry(lidarPoses, wheelPoses);
    std::cout << "Created " << wheelPosesInterp.size() << " interpolated wheel poses" << std::endl;
    
    // Step 3: Compute relative transformations
    std::cout << "\n[3/6] Computing relative transformations..." << std::endl;
    std::vector<Transform> wheelRelativeTransforms = computeRelativeTransforms(wheelPosesInterp);
    std::vector<Transform> lidarRelativeTransforms = computeRelativeTransforms(lidarPoses);
    
    size_t numConstraints = wheelRelativeTransforms.size();
    std::cout << "Generated " << numConstraints << " relative transformation constraints" << std::endl;
    
    if (numConstraints == 0) {
        std::cerr << "Error: No constraints generated. Need at least 2 poses in each trajectory." << std::endl;
        return 1;
    }
    
    // Step 4: Set up optimization problem
    std::cout << "\n[4/6] Setting up Ceres optimization problem..." << std::endl;
    
    // Initial guess for extrinsic parameters [dx, dy, d_theta]
    double extrinsic[3] = {0.0, 0.0, 0.0};
    std::cout << "Initial guess: dx=" << extrinsic[0] << " m, dy=" << extrinsic[1] 
              << " m, d_theta=" << extrinsic[2] * 180.0 / M_PI << "°" << std::endl;
    
    // Create Ceres problem
    ceres::Problem problem;
    
    // Add residual blocks for each constraint
    for (size_t i = 0; i < numConstraints; ++i) {
        ceres::CostFunction* cost_function = 
            new ceres::AutoDiffCostFunction<ExtrinsicCalibrationCostFunctor, 3, 3>(
                new ExtrinsicCalibrationCostFunctor(wheelRelativeTransforms[i], 
                                                   lidarRelativeTransforms[i]));
        
        // Add with Huber loss for robustness
        problem.AddResidualBlock(cost_function, 
                               new ceres::HuberLoss(0.1),  // Huber scale
                               extrinsic);
    }
    
    std::cout << "Added " << numConstraints << " residual blocks with Huber loss" << std::endl;
    
    // Step 5: Configure and run solver
    std::cout << "\n[5/6] Running Ceres Solver..." << std::endl;
    std::cout << "----------------------------------------------------------------------" << std::endl;
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10000;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-20;
    options.parameter_tolerance = 1e-16;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << "----------------------------------------------------------------------" << std::endl;
    
    // Print results
    std::cout << "\n======================================================================" << std::endl;
    std::cout << "OPTIMIZATION RESULTS" << std::endl;
    std::cout << "======================================================================" << std::endl;
    
    std::cout << "\nOptimized Extrinsic Parameters:" << std::endl;
    std::cout << "  dx:      " << std::fixed << std::setprecision(6) << extrinsic[0] << " m" << std::endl;
    std::cout << "  dy:      " << std::fixed << std::setprecision(6) << extrinsic[1] << " m" << std::endl;
    std::cout << "  d_theta: " << std::fixed << std::setprecision(4) 
              << extrinsic[2] * 180.0 / M_PI << "° (" 
              << std::setprecision(6) << extrinsic[2] << " rad)" << std::endl;
    
    std::cout << "\nSolver Summary:" << std::endl;
    std::cout << summary.BriefReport() << std::endl;
    
    std::cout << "\nDetailed Statistics:" << std::endl;
    std::cout << "  Termination:        " << ceres::TerminationTypeToString(summary.termination_type) << std::endl;
    std::cout << "  Initial cost:       " << std::scientific << summary.initial_cost << std::endl;
    std::cout << "  Final cost:         " << std::scientific << summary.final_cost << std::endl;
    std::cout << "  Cost reduction:     " << std::fixed << std::setprecision(2) 
              << (1.0 - summary.final_cost / summary.initial_cost) * 100.0 << "%" << std::endl;
    std::cout << "  Iterations:         " << summary.iterations.size() << std::endl;
    std::cout << "  Time:               " << std::fixed << std::setprecision(3) 
              << summary.total_time_in_seconds << " seconds" << std::endl;
    
    // Step 6: Generate corrected trajectory using proper incremental transformation
    std::cout << "\n[6/6] Generating corrected LiDAR trajectory..." << std::endl;
    std::cout << "Applying extrinsic to each incremental movement..." << std::endl;
    
    std::vector<Pose> correctedPath = generateCorrectedPath(lidarPoses, extrinsic);
    saveCorrectedTrajectory(correctedPath, "corrected_lidar_path.txt");
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "\n======================================================================" << std::endl;
    std::cout << "Calibration completed successfully!" << std::endl;
    std::cout << "Total execution time: " << duration.count() << " ms" << std::endl;
    std::cout << "======================================================================" << std::endl;
    
    return 0;
}