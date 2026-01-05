# CMakeLists.txt for application module

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(application)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../lidar_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/../pointcloud_processing
    ${CMAKE_CURRENT_SOURCE_DIR}/../segmentation
    ${CMAKE_CURRENT_SOURCE_DIR}/../measurement
    ${CMAKE_CURRENT_SOURCE_DIR}/../visualization
    ${CMAKE_CURRENT_SOURCE_DIR}/../optimization
)

# Define source files
set(APPLICATION_SOURCES
    PigMeasurementApp.cpp
)

set(APPLICATION_HEADERS
    PigMeasurementApp.h
)

# Create library
add_library(application ${APPLICATION_SOURCES} ${APPLICATION_HEADERS})

# Link libraries
target_link_libraries(application
    lidar_interface
    pointcloud_processing
    segmentation
    measurement
    visualization
    optimization
    ${PCL_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES}
)

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(application OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(application PRIVATE /W4)
else()
    target_compile_options(application PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set library properties
set_target_properties(application PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install rules (optional)
install(TARGETS application
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${APPLICATION_HEADERS}
    DESTINATION include/application
)