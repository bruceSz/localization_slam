
find_package(Eigen3 REQUIRED)


add_compile_options(-std=c++11 -g)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(example_ls
    src/example_ls.cc
)
target_link_libraries(example_ls
    ${EIGEN3_LIBRARIES}
)