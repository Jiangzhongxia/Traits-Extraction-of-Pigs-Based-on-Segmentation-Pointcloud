# CMakeLists.txt for optimization module

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(optimization)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})

# Define source files
set(OPTIMIZATION_SOURCES
    PerformanceOptimizer.cpp
)

set(OPTIMIZATION_HEADERS
    PerformanceOptimizer.h
)

# Create library
add_library(optimization ${OPTIMIZATION_SOURCES} ${OPTIMIZATION_HEADERS})

# Link libraries
target_link_libraries(optimization ${PCL_LIBRARIES})

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(optimization OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(optimization PRIVATE /W4)
else()
    target_compile_options(optimization PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set library properties
set_target_properties(optimization PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install rules (optional)
install(TARGETS optimization
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${OPTIMIZATION_HEADERS}
    DESTINATION include/optimization
)