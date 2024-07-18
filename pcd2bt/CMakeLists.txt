# declare cmake version
cmake_minimum_required(VERSION 3.5)

# set project name
project(pcd2bt)

# set C++ standard
set(CMAKE_CXX_STANDARD 14)
message(CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")

# find required packages
find_package(PCL REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)

# include directories
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${FCL_INCLUDE_DIRS})

# build collision_detector library
add_library(collision_detector INTERFACE)
target_include_directories(collision_detector INTERFACE include)
target_link_libraries(collision_detector 
                      INTERFACE
                      ${PCL_LIBRARIES}
                      ${OCTOMAP_LIBRARIES}
                      ${FCL_LIBRARIES})

# add executable
add_executable(pcd2bt src/main.cpp)
target_link_libraries(pcd2bt collision_detector)
