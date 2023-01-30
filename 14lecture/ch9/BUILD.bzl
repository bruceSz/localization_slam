

cc_binary(
    name = "fe",
    srcs = ["front_end.cc"],
    copts = [
        "-std=c++14 -I/usr/include/opencv4 -D_GLIBCXX_USE_CXX11_ABI=1",
    ],
    linkopts= [
        "-lopencv_core -lfmt -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -ljpeg -lopencv_features2d  ",
    ],
    deps = [
        ":common",
    ]
)

 