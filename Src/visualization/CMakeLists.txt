# CMakeLists.txt for visualization module

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(visualization)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../measurement)  # 包含测量模块头文件

# Define source files
set(VISUALIZATION_SOURCES
    VisualizationManager.cpp
)

set(VISUALIZATION_HEADERS
    VisualizationManager.h
)

# Create library
add_library(visualization ${VISUALIZATION_SOURCES} ${VISUALIZATION_HEADERS})

# Link libraries
target_link_libraries(visualization
    ${PCL_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES}
    measurement  # 链接测量模块
)

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(visualization OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(visualization PRIVATE /W4)
else()
    target_compile_options(visualization PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set library properties
set_target_properties(visualization PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install rules (optional)
install(TARGETS visualization
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${VISUALIZATION_HEADERS}
    DESTINATION include/visualization
)