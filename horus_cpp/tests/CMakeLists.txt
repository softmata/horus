cmake_minimum_required(VERSION 3.20)
project(horus_cpp_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HORUS_RUST_TARGET_DIR "" CACHE PATH "Path to the Rust target dir containing libhorus_cpp")
if(NOT HORUS_RUST_TARGET_DIR)
    set(HORUS_RUST_TARGET_DIR "${CMAKE_SOURCE_DIR}/../../target/debug")
endif()

include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

function(horus_add_gtest NAME SRC)
    add_executable(${NAME} ${SRC})
    target_compile_options(${NAME} PRIVATE -fext-numeric-literals)
    target_include_directories(${NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../include)
    target_link_directories(${NAME} PRIVATE ${HORUS_RUST_TARGET_DIR})
    target_link_libraries(${NAME} PRIVATE GTest::gtest_main horus_cpp pthread dl m)
    gtest_discover_tests(${NAME} DISCOVERY_TIMEOUT 60)
endfunction()

# Same as horus_add_gtest but tags tests with LABELS="nightly" so default
# ctest runs skip them — nightly-only tests (soak) live here.
function(horus_add_nightly_gtest NAME SRC)
    add_executable(${NAME} ${SRC})
    target_compile_options(${NAME} PRIVATE -fext-numeric-literals)
    target_include_directories(${NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../include)
    target_link_directories(${NAME} PRIVATE ${HORUS_RUST_TARGET_DIR})
    target_link_libraries(${NAME} PRIVATE GTest::gtest_main horus_cpp pthread dl m)
    gtest_discover_tests(${NAME} DISCOVERY_TIMEOUT 60
        PROPERTIES LABELS "nightly")
endfunction()

horus_add_gtest(cpp_e2e_test        cpp_e2e_test.cpp)
horus_add_gtest(cpp_ergonomic_e2e   cpp_ergonomic_e2e.cpp)
horus_add_gtest(cpp_full_api_test   cpp_full_api_test.cpp)
horus_add_gtest(cpp_user_api_test   cpp_user_api_test.cpp)
horus_add_gtest(cpp_stress_test     cpp_stress_test.cpp)
horus_add_gtest(cpp_all_types_test  cpp_all_types_test.cpp)
horus_add_gtest(cpp_fault_injection cpp_fault_injection.cpp)

# Nightly-only: 2-hour soak. Default ctest excludes via label filter.
horus_add_nightly_gtest(cpp_soak_test cpp_soak_test.cpp)

# cpp_benchmark — plain perf binary (no gtest).
add_executable(cpp_benchmark cpp_benchmark.cpp)
target_compile_options(cpp_benchmark PRIVATE -O2 -fext-numeric-literals)
target_include_directories(cpp_benchmark PRIVATE ${CMAKE_SOURCE_DIR}/../include)
target_link_directories(cpp_benchmark PRIVATE ${HORUS_RUST_TARGET_DIR})
target_link_libraries(cpp_benchmark PRIVATE horus_cpp pthread dl m)

# cpp_live_node — long-running CLI for introspection tests (takes argv).
add_executable(cpp_live_node cpp_live_node.cpp)
target_compile_options(cpp_live_node PRIVATE -fext-numeric-literals)
target_include_directories(cpp_live_node PRIVATE ${CMAKE_SOURCE_DIR}/../include)
target_link_directories(cpp_live_node PRIVATE ${HORUS_RUST_TARGET_DIR})
target_link_libraries(cpp_live_node PRIVATE horus_cpp pthread dl m)
