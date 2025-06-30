# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(FunnelPathFinder)

# Set the C++ standard to C++17 (a good modern choice)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the OpenCV library.
find_package(OpenCV REQUIRED)

# Find the GEOS library.
find_library(GEOS_LIBRARY NAMES geos_c)
if(NOT GEOS_LIBRARY)
    message(FATAL_ERROR "libgeos_c not found!")
endif()

# Define our executable.
add_executable(pathfinder src/main.cpp)

# Add the include directory
target_include_directories(pathfinder
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include # Assumes json.hpp is in ./include
)

# Link our executable against the libraries we found.
target_link_libraries(pathfinder
    PRIVATE
    ${OpenCV_LIBS}
    ${GEOS_LIBRARY}
)