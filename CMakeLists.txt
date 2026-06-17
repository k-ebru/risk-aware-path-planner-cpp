cmake_minimum_required(VERSION 3.14)
project(RiskAwarePathPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(planner
    src/main.cpp
    src/grid.cpp
    src/astar.cpp
    src/rrt.cpp
    src/smoothing.cpp
)

target_include_directories(planner PRIVATE src)
