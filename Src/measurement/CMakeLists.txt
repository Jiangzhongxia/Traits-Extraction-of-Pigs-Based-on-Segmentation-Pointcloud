# CMakeLists.txt for measurement module

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(measurement)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})

# Define source files
set(MEASUREMENT_SOURCES
    BodyMeasurementCalculator.cpp
)

set(MEASUREMENT_HEADERS
    BodyMeasurementCalculator.h
)

# Create library
add_library(measurement ${MEASUREMENT_SOURCES} ${MEASUREMENT_HEADERS})

# Link libraries
target_link_libraries(measurement ${PCL_LIBRARIES})

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(measurement OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(measurement PRIVATE /W4)
else()
    target_compile_options(measurement PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set library properties
set_target_properties(measurement PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install rules (optional)
install(TARGETS measurement
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${MEASUREMENT_HEADERS}
    DESTINATION include/measurement
)