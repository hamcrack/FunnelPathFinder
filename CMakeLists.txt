# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(FunnelPathFinder)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the OpenCV library
find_package(OpenCV REQUIRED)

# --- Updated GEOS Section ---
# Find the main GEOS C++ include directory
find_path(GEOS_INCLUDE_DIR geos/geom/Point.h)
if(NOT GEOS_INCLUDE_DIR)
    message(FATAL_ERROR "GEOS C++ headers not found! Make sure libgeos-dev is installed.")
endif()

# Find the GEOS C++ library (libgeos)
find_library(GEOS_CPP_LIBRARY NAMES geos)
if(NOT GEOS_CPP_LIBRARY)
    message(FATAL_ERROR "libgeos (C++ library) not found!")
endif()

# Find the GEOS C library (for init/finishGEOS)
find_library(GEOS_C_LIBRARY NAMES geos_c)
if(NOT GEOS_C_LIBRARY)
    message(FATAL_ERROR "libgeos_c (C library) not found!")
endif()

# Define our executable.
add_executable(pathfinder src/main.cpp)

# Add the include directory
target_include_directories(pathfinder
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include # Assumes json.hpp is in ./include
)

# Add the GEOS C++ include directory to our target's include paths
target_include_directories(pathfinder PRIVATE ${GEOS_INCLUDE_DIR})

# Link our executable against the libraries we found.
target_link_libraries(pathfinder
    PRIVATE
    ${OpenCV_LIBS}
    ${GEOS_CPP_LIBRARY} # Link the C++ library
    ${GEOS_C_LIBRARY}   # Link the C library
)