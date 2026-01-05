# CMakeLists.txt for segmentation module

# Set minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Define project
project(segmentation)

# Find required packages
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Find ONNX Runtime (需要预先安装)
# find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h)
# find_library(ONNXRUNTIME_LIBRARY onnxruntime)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
# include_directories(${ONNXRUNTIME_INCLUDE_DIR})

# Define source files
set(SEGMENTATION_SOURCES
    ONNXSegmenter.cpp
)

set(SEGMENTATION_HEADERS
    ONNXSegmenter.h
)

# Create library
add_library(segmentation ${SEGMENTATION_SOURCES} ${SEGMENTATION_HEADERS})

# Link libraries
target_link_libraries(segmentation ${PCL_LIBRARIES})

# Link ONNX Runtime if found
# if(ONNXRUNTIME_LIBRARY)
#     target_link_libraries(segmentation ${ONNXRUNTIME_LIBRARY})
# endif()

# If OpenMP is found, use it
if(OPENMP_FOUND)
    target_link_libraries(segmentation OpenMP::OpenMP_CXX)
endif()

# Compiler-specific options
if(MSVC)
    target_compile_options(segmentation PRIVATE /W4)
else()
    target_compile_options(segmentation PRIVATE -Wall -Wextra -std=c++17)
endif()

# Set library properties
set_target_properties(segmentation PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install rules (optional)
install(TARGETS segmentation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${SEGMENTATION_HEADERS}
    DESTINATION include/segmentation
)