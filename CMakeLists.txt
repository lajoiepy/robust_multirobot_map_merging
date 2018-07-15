cmake_minimum_required(VERSION 3.1)
project(robust_multirobot_map_merging C CXX)


# Set this project's configuration
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # We require C++ 14

#set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# Set build type to 'RelWithDebInfo' if one was not specified by the user
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS RelWithDebInfo Release Debug MinSizeRel)
  message(STATUS "Setting build type to ${CMAKE_BUILD_TYPE}, as none was specified\n")
else()
  message(STATUS "Building in ${CMAKE_BUILD_TYPE} mode\n")
endif()

# Directory for built libraries
#set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib CACHE PATH "The directory in which to place the SE-Sync library built by this project")
# Directory for built executables
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin CACHE PATH "The directory in which to place executables built by this project")

# BUILD CONFIGURATIONS
# Enable faster instruction sets (SIMD/AVX)
set(ENABLE_FAST_INSTRUCTIONS ON CACHE BOOL "Enable faster instruction sets (SIMD/AVX)")
# Enable OpenMP (if available)
set(ENABLE_OPENMP ON CACHE BOOL "Enable OpenMP (if available)")
# Enable code profiling using gperftools
set(ENABLE_PROFILING OFF CACHE BOOL "Enable code profiling using gperftools")

# Add the .cmake files that ship with Eigen3 to the CMake module path (useful for finding other stuff)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/eigen/cmake" CACHE STRING "The CMake module path used for this project")

if(${ENABLE_FAST_INSTRUCTIONS})
message(STATUS "Enabling SIMD/AVX instruction sets")
add_definitions(-march=native)
endif()

if(${ENABLE_PROFILING})
message(STATUS "Enabling code profiling using Google Performance Tools")
set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} -lprofiler)
endif()

message(STATUS "")


# SESYNC DEPENDENCIES

# FIND EIGEN3
set( ENV{EIGEN3_ROOT_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/eigen)
find_package(Eigen3 3.3.3 REQUIRED)
if(EIGEN3_FOUND)
message(STATUS "Found Eigen3 library (version ${EIGEN3_VERSION})")
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}\n")
else()
message(STATUS "Eigen library not found!")
endif()
find_package(SPQR REQUIRED)
find_package(Cholmod REQUIRED)
find_package(BLAS REQUIRED)

# SE-Sync library
#Find SE_Sync library
set(SE_SYNC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/SE-Sync/ CACHE PATH "Path to top-level directory of SE-Sync (the one containing CMakeLists.txt)")
# Run this project's CMakeLists file
add_subdirectory(${SE_SYNC_DIR})


find_package(catkin REQUIRED
    geometry_msgs
    roscpp
    tf2
    mrpt_bridge
    )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} graph_utils
  CATKIN_DEPENDS roscpp geometry_msgs tf2 mrpt_bridge
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/third_parties/fast_max-clique_finder/src/
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

add_executable(${PROJECT_NAME}_node test/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   graph_utils
   fast_max-clique_finder
   SESync
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