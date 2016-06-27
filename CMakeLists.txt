cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)

set(POD_NAME benchmark_dynamics)
include(cmake/pods.cmake)

add_executable(benchmark_dynamics benchmark_dynamics.cpp)
target_link_libraries(benchmark_dynamics drakeRBM drakeRBSystem)
add_dependencies(benchmark_dynamics drakeRBM drakeRBSystem)
pods_use_pkg_config_packages(benchmark_dynamics drake-rbm eigen3)
