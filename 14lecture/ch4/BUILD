cc_binary(
    name = "traj_error",
    srcs   = ["traj_error.cc"],
    copts = [
        "-std=c++14 -I/usr/include/eigen3",
    ],
    linkopts= [
        #"-L/usr/local/lib   -lpango_core -lpango_display -lpango_geometry -lpango_glgeometry 
        # -lpango_image -lpango_opengl -lpango_packetstream -lpango_plot -lpango_python -lpango_scene 
        # -lpango_tools -lpango_vars -lpango_video -lpango_windowing",
        "-L /usr/local/lib  -lfmt -lpango_opengl  -lOpenGL -lGLEW   -lpango_display    ",
        

    ]
)

cc_binary(
    name = "use_sophus",
    srcs = ["use_sophus.cc"],
    copts = [
        "-std=c++14 -I/usr/include/eigen3",
    ],
    linkopts= [
        "-L /usr/local/lib  -lfmt -lpango_opengl  -lOpenGL -lGLEW   -lpango_display    ",
    ]

)