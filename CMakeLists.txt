# Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>
cmake_minimum_required(VERSION 3.1)
project(robust_multirobot_map_merging C CXX)

## Compile as C++14
# BUILD CONFIGURATIONS
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Enable faster instruction sets (SIMD/AVX)
set(ENABLE_FAST_INSTRUCTIONS ON CACHE BOOL "Enable faster instruction sets (SIMD/AVX)")

if(${ENABLE_FAST_INSTRUCTIONS})
message(STATUS "Enabling SIMD/AVX instruction sets")
add_definitions(-march=native)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync)

find_library(SESync sesync REQUIRED)

# Required ROS packages
find_package(catkin REQUIRED
    geometry_msgs
    mrpt_bridge
    )

# Required exernal library
find_package(MRPT REQUIRED)
message(STATUS "Found MRPT: " ${MRPT_VERSION})
if("${MRPT_VERSION}" VERSION_LESS "1.9.9")
  # MRPT<2.0
  find_package(MRPT REQUIRED base)
else()
  # MRPT>=2.0
  set (CMAKE_CXX_STANDARD 14) # C++14 actually only needed when building
  find_package(MRPT REQUIRED poses)
endif()

# Catkin package definition
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pairwise_consistency graph_utils
  CATKIN_DEPENDS geometry_msgs mrpt_bridge
)

# Overall include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/fast_max-clique_finder/src/
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/SE-Sync/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/Optimization/include/
)

# Paiwise Consistency computation library
add_library(pairwise_consistency
    src/pairwise_consistency/pairwise_consistency.cpp
)
target_link_libraries(pairwise_consistency
   ${catkin_LIBRARIES}
   graph_utils
)
add_dependencies(pairwise_consistency ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Graph utils library
add_library(graph_utils
    src/graph_utils/graph_utils_functions.cpp
)
target_link_libraries(graph_utils
   ${catkin_LIBRARIES}
)

# Maximum Clique Solver
add_library(fast_max-clique_finder
    src/third_parties/fast_max-clique_finder/src/findClique.h
    src/third_parties/fast_max-clique_finder/src/graphIO.h
    src/third_parties/fast_max-clique_finder/src/findClique.cpp
    src/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp 
    src/third_parties/fast_max-clique_finder/src/utils.cpp 
    src/third_parties/fast_max-clique_finder/src/graphIO.cpp
)
target_compile_options(fast_max-clique_finder PRIVATE -w)

# Robot local map library
add_library(robot_local_map
src/robot_local_map/robot_measurements.cpp
src/robot_local_map/robot_local_map.cpp
)
target_link_libraries(robot_local_map
   ${catkin_LIBRARIES}
   graph_utils
)

# Global map solver library
add_library(global_map_solver
src/global_map_solver/global_map_solver.cpp
)
target_link_libraries(global_map_solver
   ${catkin_LIBRARIES}
   graph_utils
   pairwise_consistency
   fast_max-clique_finder
   SESync
)

# Example program
add_executable(${PROJECT_NAME}_node examples/example_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   graph_utils
   robot_local_map
   global_map_solver
   SESync
)
