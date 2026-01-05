# CMakeLists.txt for pointcloud_processing module

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(pointcloud_processing)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})

# Define source files
set(POINTCLOUD_PROCESSING_SOURCES
    PointCloudProcessor.cpp
)

set(POINTCLOUD_PROCESSING_HEADERS
    PointCloudProcessor.h
)

# Create library
add_library(pointcloud_processing ${POINTCLOUD_PROCESSING_SOURCES} ${POINTCLOUD_PROCESSING_HEADERS})

# Link libraries
target_link_libraries(pointcloud_processing ${PCL_LIBRARIES})

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(pointcloud_processing OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(pointcloud_processing PRIVATE /W4)
else()
    target_compile_options(pointcloud_processing PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set library properties
set_target_properties(pointcloud_processing PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install rules (optional)
install(TARGETS pointcloud_processing
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${POINTCLOUD_PROCESSING_HEADERS}
    DESTINATION include/pointcloud_processing
)