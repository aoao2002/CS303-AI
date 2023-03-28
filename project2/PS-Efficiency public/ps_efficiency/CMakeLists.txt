cmake_minimum_required(VERSION 2.4)
project(psefficiency_experiment)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -lemon")

set(SOURCE_FILES psefficiency_experiment.cpp)
add_executable(psefficiency_experiment ${SOURCE_FILES})
