cc_binary(
    name = "img_basic",
    srcs = ["image_basic.cc"],
    copts = [
        "-std=c++14 -I/usr/include/opencv4 -D_GLIBCXX_USE_CXX11_ABI=1",
    ],
    linkopts= [
        "-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -ljpeg",
    ]

    

)


cc_binary(
    name = "undistorted_img",
    srcs = ["undistorted_img.cc"],
     copts = [
        "-std=c++14 -I/usr/include/opencv4 -D_GLIBCXX_USE_CXX11_ABI=1",
    ],
    linkopts= [
        "-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs ",
    ]

)

cc_binary(
    name = "stereo",
    srcs = ["stereo.cc"],
    copts = [
        "-std=c++14 -I/usr/include/opencv4 -D_GLIBCXX_USE_CXX11_ABI=1 -I/usr/include/eigen3 ",
    ],
    linkopts= [
        "-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lpango_opengl -L /usr/local/lib  -lOpenGL -lGLEW   -lpango_display  ",
    ]


)

cc_binary(
    name = "jm"
    srcs = [ "jm.cc"],
    copts = [
        "-std=c++14 -I/usr/include/opencv4 -D_GLIBCXX_USE_CXX11_ABI=1 -I/usr/include/eigen3 ",
    ],
    linkopts= [
        "-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lpango_opengl -L /usr/local/lib  -lOpenGL -lGLEW   -lpango_display  ",
    ]

)