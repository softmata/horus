# HORUS C++ Examples — CMakeLists.txt
#
# Build all examples:
#
#   cmake -S . -B build \
#       -DHORUS_CPP_LIB=/path/to/horus/target/debug/libhorus_cpp.a \
#       -DHORUS_CPP_INCLUDE=/path/to/horus/horus_cpp/include
#   cmake --build build
#
# Build the static library first if you do not have one:
#
#   cargo build -p horus_cpp
#
# From a checkout of this repository the paths above are found automatically.
#
# This used to say `find_package(horus QUIET)` and link only `if(horus_FOUND)`.
# HORUS does not install a CMake package config — `horus build` passes the
# library and include paths in with -D (see run_cpp.rs) — so find_package never
# succeeded, nothing was linked, and every example failed with a wall of
# undefined references (82 of them) instead of a message saying what was
# missing.

cmake_minimum_required(VERSION 3.20)
project(horus_cpp_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Locate the HORUS C++ bindings ─────────────────────────────────────────
#
# Prefer what the caller passed; otherwise look where a repository checkout
# puts them.

set(_repo_root "${CMAKE_CURRENT_SOURCE_DIR}/../..")

if(NOT HORUS_CPP_INCLUDE)
    set(HORUS_CPP_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
endif()

if(NOT HORUS_CPP_LIB)
    find_library(HORUS_CPP_LIB
        # libhorus_cpp.a first: a static link means the examples run without
        # LD_LIBRARY_PATH pointing back into the build tree.
        NAMES libhorus_cpp.a horus_cpp
        PATHS "${_repo_root}/target/debug" "${_repo_root}/target/release"
        NO_DEFAULT_PATH)
endif()

if(NOT HORUS_CPP_LIB)
    message(FATAL_ERROR
        "libhorus_cpp not found.\n"
        "Build it with:  cargo build -p horus_cpp\n"
        "or pass it explicitly:  -DHORUS_CPP_LIB=/path/to/libhorus_cpp.a\n"
        "Without it these examples compile but cannot link.")
endif()

message(STATUS "HORUS C++ library:  ${HORUS_CPP_LIB}")
message(STATUS "HORUS C++ includes: ${HORUS_CPP_INCLUDE}")

# ── Examples ──────────────────────────────────────────────────────────────
#
# pub_sub_demo was missing from this list, so it was never compiled by anything.

set(EXAMPLES
    pub_sub_demo
    obstacle_avoidance
    camera_publisher
    multi_node
    transform_frames
    params_demo
)

foreach(example ${EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_include_directories(${example} PRIVATE ${HORUS_CPP_INCLUDE})
    # pthread/dl/m are required by the horus_cpp static library.
    target_link_libraries(${example} PRIVATE
        ${HORUS_CPP_LIB} pthread ${CMAKE_DL_LIBS} m)
endforeach()
