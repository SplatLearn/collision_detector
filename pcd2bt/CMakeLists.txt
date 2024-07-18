# declare cmake version
cmake_minimum_required(VERSION 3.5)

# set project name
project(pcd2bt)

# set C++ standard
set(CMAKE_CXX_STANDARD 14)
message(CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")

list(PREPEND CMAKE_MODULE_PATH "/usr/local/share/octomap")

# find required packages
find_package(PCL REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)

message(OCTOMAP_INCLUDE_DIRS="${OCTOMAP_INCLUDE_DIRS}")

# include directories
include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${FCL_INCLUDE_DIRS})

message(OCTOMAP_LIBRARIES="${OCTOMAP_LIBRARIES}")

# link directories
# link_directories(${PCL_LIBRARY_DIRS})
# link_directories(${OCTOMAP_LIBRARY_DIRS})

# add executable
add_executable(pcd2bt src/main.cpp)
target_link_libraries(pcd2bt ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${FCL_LIBRARIES})
