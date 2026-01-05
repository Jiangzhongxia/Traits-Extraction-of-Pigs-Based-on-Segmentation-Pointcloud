# Main CMakeLists.txt for the pig measurement system

cmake_minimum_required(VERSION 3.10)
project(pig_measurement_system)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Set PCL-specific flags
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH ${PCL_ROOT})

# Add subdirectories for each module
add_subdirectory(lidar_interface)
add_subdirectory(pointcloud_processing)
add_subdirectory(segmentation)
add_subdirectory(measurement)
add_subdirectory(visualization)
add_subdirectory(optimization)
add_subdirectory(application)
add_subdirectory(tests)

# Include directories for the entire project
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/lidar_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/pointcloud_processing
    ${CMAKE_CURRENT_SOURCE_DIR}/segmentation
    ${CMAKE_CURRENT_SOURCE_DIR}/measurement
    ${CMAKE_CURRENT_SOURCE_DIR}/visualization
    ${CMAKE_CURRENT_SOURCE_DIR}/optimization
    ${CMAKE_CURRENT_SOURCE_DIR}/application
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create main executable (optional)
# This would be the final integrated application
add_executable(pig_measurement_main main.cpp)

# Link all modules to the main executable
target_link_libraries(pig_measurement_main
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
    target_link_libraries(pig_measurement_main OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(pig_measurement_main PRIVATE /W4)
else()
    target_compile_options(pig_measurement_main PRIVATE -Wall -Wextra)
endif()

# Install rules
install(TARGETS pig_measurement_main
    RUNTIME DESTINATION bin
)

# Install configuration and model files (if they exist)
# file(GLOB config_files "config/*.json")
# if(config_files)
#     install(FILES ${config_files} DESTINATION config)
# endif()
#
# file(GLOB model_files "models/*.onnx")
# if(model_files)
#     install(FILES ${model_files} DESTINATION models)
# endif()