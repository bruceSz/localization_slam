
add_executable(pcl_write
    pcl_write.cc
)

target_link_libraries(pcl_write
    ${PCL_LIBRARIES}
)