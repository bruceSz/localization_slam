load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "coor_transform",
    srcs = ["coor_transform.cc"]

)

cc_binary(
    name = "plot_trajectory",
    deps = [ "plot_trajectory_lib"]
)
cc_library(
    name = "plot_trajectory_lib",
    srcs = ["plot_trajectory.cc"],
    copts = [
        "-std=c++14 ",
        #"-std=c++14 -g -Wall -fPIC -O3 -Wno-unused-function -Wno-unused-variable -Wno-maybe-uninitialized -Wno-deprecated-declarations",
    ],
    linkopts= [
        #"-L/usr/local/lib   -lpango_core -lpango_display -lpango_geometry -lpango_glgeometry 
        # -lpango_image -lpango_opengl -lpango_packetstream -lpango_plot -lpango_python -lpango_scene 
        # -lpango_tools -lpango_vars -lpango_video -lpango_windowing",
        "-L /usr/local/lib   -lpango_opengl  -lOpenGL -lGLEW   -lpango_display    ",
    ]
    

)

cc_binary(
    name = "eigen_basic",
    srcs = ["eigen_basic.cc"]   
)

cc_binary(
    name = "eigen_geometry",
    srcs = ["eigen_geometry.cc"]
)

cc_binary(
    name = "viz_geometry",
    srcs = ["viz_geometry.cc"],
    copts = [
        "-std=c++14 -I/usr/include/eigen3",
    ],
    linkopts= [
        
        "-L /usr/local/lib  -lGL  -lpango_opengl  -lOpenGL -lGLEW   -lpango_display  -lpango_vars -lpango_core  ",
    ]
)