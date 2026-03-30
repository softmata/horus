# HORUS C++ Examples — CMakeLists.txt
#
# Build all examples:
#   mkdir build && cd build && cmake .. && make

cmake_minimum_required(VERSION 3.20)
project(horus_cpp_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find HORUS
find_package(horus QUIET)

# Example executables
set(EXAMPLES
    obstacle_avoidance
    camera_publisher
    multi_node
    transform_frames
    params_demo
)

foreach(example ${EXAMPLES})
    add_executable(${example} ${example}.cpp)
    if(horus_FOUND)
        target_link_libraries(${example} PRIVATE horus::horus)
    endif()
    # Include the horus headers
    target_include_directories(${example} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
    )
endforeach()
