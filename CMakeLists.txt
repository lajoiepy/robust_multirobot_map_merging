cmake_minimum_required(VERSION 2.8.3)
project(robust_multirobot_map_merging)

## Compile as C++14
add_compile_options(-std=c++14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED
    geometry_msgs
    roscpp
    tf2
    mrpt_bridge
    )

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


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} graph_utils
  CATKIN_DEPENDS roscpp geometry_msgs tf2 mrpt_bridge
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/fast_max-clique_finder/src/
  #${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/SE-Sync/include/
  #${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/spectra/include/
  #${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/Optimization/include/
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/pairwise_consistency/pairwise_consistency.cpp
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   graph_utils
)

add_library(graph_utils
    src/graph_utils/graph_utils.cpp
)

add_library(fast_max-clique_finder
    src/third_parties/fast_max-clique_finder/src/findClique.h
    src/third_parties/fast_max-clique_finder/src/graphIO.h
    src/third_parties/fast_max-clique_finder/src/findClique.cpp
    src/third_parties/fast_max-clique_finder/src/findCliqueHeu.cpp 
    src/third_parties/fast_max-clique_finder/src/utils.cpp 
    src/third_parties/fast_max-clique_finder/src/graphIO.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node test/example_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   graph_utils
   fast_max-clique_finder
)
