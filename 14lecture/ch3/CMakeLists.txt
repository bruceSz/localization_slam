find_package(Eigen3)
message("message: " ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(hw3_7_tranform
    hw3_7_transform.cc
)