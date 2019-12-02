cmake_minimum_required(VERSION 3.10)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.8 REQUIRED)
message("------ --------message: " ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_subdirectory(ch3)
add_subdirectory(ch5)