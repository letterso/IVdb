cmake_minimum_required(VERSION 3.10)
project(vox_comparison)

set(CMAKE_CXX_STANDARD 17)

include(cmake/static_check.cmake)

# Find required packages
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Define source files
set(SOURCES
    ivdb/ivdb.cpp
    main.cpp
)

# Add executable
add_executable(vox_comparison ${SOURCES})
target_link_libraries(vox_comparison ${PCL_LIBRARIES})
target_include_directories(vox_comparison PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)