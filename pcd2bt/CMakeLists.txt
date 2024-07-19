# declare cmake version
cmake_minimum_required(VERSION 3.15...3.29)

# set project name
project(pcd2bt)

# set C++ standard
set(CMAKE_CXX_STANDARD 14)
message(CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")

# find required packages
find_package(PCL REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)
set(BIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

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

# build python bindings
pybind11_add_module(pybind_collision_detector src/pybind_collision_detector.cpp)
target_link_libraries(pybind_collision_detector PRIVATE collision_detector)
install(TARGETS pybind_collision_detector DESTINATION .)

# add executable
add_executable(pcd2bt src/main.cpp)
target_link_libraries(pcd2bt collision_detector)

# add executable
add_executable(test_box_box_collision test/test_box_box_collision.cpp)
target_link_libraries(test_box_box_collision ${FCL_LIBRARIES})
