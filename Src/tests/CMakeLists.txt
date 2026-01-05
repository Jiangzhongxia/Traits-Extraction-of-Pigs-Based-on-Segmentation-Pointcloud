# CMakeLists.txt for tests

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(pig_measurement_tests)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../lidar_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/../pointcloud_processing
    ${CMAKE_CURRENT_SOURCE_DIR}/../segmentation
    ${CMAKE_CURRENT_SOURCE_DIR}/../measurement
    ${CMAKE_CURRENT_SOURCE_DIR}/../visualization
    ${CMAKE_CURRENT_SOURCE_DIR}/../optimization
    ${CMAKE_CURRENT_SOURCE_DIR}/../application
    ${PCL_INCLUDE_DIRS}
)

# Define executable
add_executable(test_main test_main.cpp)

# Link libraries
target_link_libraries(test_main
    lidar_interface
    pointcloud_processing
    segmentation
    measurement
    visualization
    optimization
    application
    ${PCL_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES}
)

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(test_main OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(test_main PRIVATE /W4)
else()
    target_compile_options(test_main PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set properties
set_target_properties(test_main PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Copy required files for testing
# This would typically include model files, test data, etc.
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data DESTINATION ${CMAKE_BINARY_DIR})