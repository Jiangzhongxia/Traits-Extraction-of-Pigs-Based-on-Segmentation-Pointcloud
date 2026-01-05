# CMakeLists.txt for lidar_interface module

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(lidar_interface)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})

# Define source files
set(LIDAR_INTERFACE_SOURCES
    LivoxLidarInterface.cpp
)

set(LIDAR_INTERFACE_HEADERS
    LivoxLidarInterface.h
)

# Create library
add_library(lidar_interface ${LIDAR_INTERFACE_SOURCES} ${LIDAR_INTERFACE_HEADERS})

# Link libraries
target_link_libraries(lidar_interface ${PCL_LIBRARIES})

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(lidar_interface OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(lidar_interface PRIVATE /W4)
else()
    target_compile_options(lidar_interface PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set library properties
set_target_properties(lidar_interface PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install rules (optional)
install(TARGETS lidar_interface
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${LIDAR_INTERFACE_HEADERS}
    DESTINATION include/lidar_interface
)