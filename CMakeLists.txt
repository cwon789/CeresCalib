cmake_minimum_required(VERSION 3.10)
project(LidarExtrinsicCalibration)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Add executables
add_executable(extrinsic_calibrator main.cpp)
add_executable(extrinsic_calibrator_corrected main_corrected.cpp)

# Link libraries
target_link_libraries(extrinsic_calibrator
    ${CERES_LIBRARIES}
)

target_link_libraries(extrinsic_calibrator_corrected
    ${CERES_LIBRARIES}
)

# Set optimization flags for release build
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
endif()

# Print build information
message(STATUS "Ceres found: ${CERES_FOUND}")
message(STATUS "Ceres version: ${CERES_VERSION}")
message(STATUS "Ceres include dirs: ${CERES_INCLUDE_DIRS}")
message(STATUS "Ceres libraries: ${CERES_LIBRARIES}")